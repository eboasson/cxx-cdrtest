#
# Copyright(c) 2019, 2021 ADLINK Technology Limited and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License v. 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
# v. 1.0 which is available at
# http://www.eclipse.org/org/documents/edl-v10.php.
#
# SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
#

=begin howto

perl -w ~/C/cxx-cdrtest.pl && \
  ~/C/cdds/CC/install/bin/idlc -l lib/libcycloneddsidlcxx.dylib xxx-types.idl && \
  ~/C/cdds/CC/install/bin/idlc xxx-types.idl && \
  clang -I ~/C/cdds/CC/install/include -g -Wall -c xxx-types.c && \
  clang++ -std=gnu++17 -I src/ddscxx/include -I ../src/ddscxx/include -I ~/C/cdds/CC/install/include -I /Users/erik/C/iceoryx/install/include -L lib -L ~/C/cdds/CC/lib -rpath $PWD/lib -rpath ~/C/cdds/CC/lib -g -Wall -Wno-switch-bool xxx*.cpp xxx-types.o -o xxx -lddscxx -lddsc

=end howto
=cut

# ls -l xxx*
#   -rw-r--r--  1 erik  staff    8882  5 Jul 16:23 xxx-types.c
#   -rw-r--r--  1 erik  staff   11040  5 Jul 16:23 xxx-types.h
#   -rw-r--r--  1 erik  staff  150067  5 Jul 16:23 xxx-types.hpp
#   -rw-r--r--  1 erik  staff    3587  5 Jul 16:21 xxx-types.idl
#   -rw-r--r--  1 erik  staff   67596  5 Jul 16:21 xxx.cpp
#
##

use strict;
use warnings;
use Config;
# assuming 64-bit integers support present in this perl build
die unless $Config{use64bitint} eq "define";
use Getopt::Std;
#$SIG{__DIE__} = sub { $DB::single = 1; die @_; };

use Data::Dumper;
$Data::Dumper::Terse = 1;
$Data::Dumper::Useqq = 1;

# OPTIONS
# -a       annotate with @data_representation(XCDR2)
# -c FILE  read corpus of types to not generate from FILE;
#          append generated types to it **EVALS FILE**
# -i CNT   generate CNT random instances of each generated type
# -l DEPTH limit type generation depth to DEPTH
# -n CNT   generate CNT types, defaults to 1 if corpus file given,
#          else to 100
my %opts;
getopts('ac:i:l:n:', \%opts) or die "$0 invalid options given";
my $force_xcdr2_annot = exists $opts{a} ? 1 : 0;
my ($corpus_file, $ntypes);
if (exists $opts{c}) {
  $corpus_file = $opts{c};
  $ntypes = exists $opts{n} ? $opts{n} : 1;
} else {
  $ntypes = exists $opts{n} ? $opts{n} : 100;
}
my $nvalues = exists $opts{i} ? $opts{i} : 50;
my $maxlevel = exists $opts{l} ? $opts{l} : 10;
# ARGUMENTS: SEED
my $seed;
if (@ARGV) {
  $seed = $ARGV[0];
  srand $ARGV[0];
} else {
  $seed = srand;
}
print "seed $seed\n";

# WORKAROUNDS

# old workarounds for problems that appear to be completely fixed
my $workaround_no_opt_seq_string = 0; # C serializer issue
my $workaround_no_union = 0; # was some C++ issue, don't remember the details
# std::variant troubles (https://github.com/eclipse-cyclonedds/cyclonedds-cxx/issues/127)
# mostly solved, but not quite
my $workaround_unique_types_in_union = 0;
my $workaround_no_bitmask_in_union = 1;
my $workaround_no_enum_in_union = 1;
# arrays are a different problem I think, but I don't remember the details
# https://github.com/eclipse-cyclonedds/cyclonedds-cxx/issues/132
my $workaround_no_arrays_in_union = 1;
# C compiler backend must put "default" label last in VM, but it doesn't
# https://github.com/eclipse-cyclonedds/cyclonedds/issues/874
my $workaround_default_always_last = 1;
# annoying number of warnings from IDLC if enums non-consecutive
my $workaround_no_random_enums = 1;
# looks to me like C++ serialiser doesn't respect @bit_bound on enum
my $workaround_only_32bit_enums = 0;
# typedef boolean a02024;
# typedef sequence<a02024, 1> a02023;
# @mutable struct a02021 {
#   a02023 a02022;
#   a02026 a02025;
# };
# =>
# ./xxx-types.hpp:106597:12: error: no matching function for call to 'read__x_a02024'
#      if (!read__x_a02024(streamer, instance[i_1]))
#           ^~~~~~~~~~~~~~
my $workaround_no_silly_typedefs = 0;
my $workaround_no_bitmask = 0;

#########################################################################################

sub normprobs {
  my @ps = @_;
  my (@xs, $sum);
  for (@ps) { $sum += $_; push @xs, $sum; }
  map { $_ / $sum } @xs;
}

# - u... used to just be integers of various sizes, this script grew a bit into a
#   mess and now they are the primitive types (plus string and except "char", for now)
# - typedefs don't count towards the depth of the type
my @types = qw(u0 u1 u2 u3 u4 u5 u6 u7 bm enu typedef seq ary str uni);

sub disabletype {
  my ($psref, $name) = @_;
  for (my $i = 0; $i < @types; $i++) {
    next unless $name eq $types[$i];
    $psref->[$i] = 0;
    return;
  }
  die "type $name not found";
}

# once the maximum depth is reached, it only generates types in
# $types[0 .. $maxsimpletype], those are the leafs in the type tree
my $maxsimpletype = do { my $m = 0; $m++ while $types[$m+1] =~ /^(?:u\d+|bm|enu)$/; $m };

# type names for the primitive types
my @idltype = ("boolean", "octet", "unsigned short", "unsigned long", "unsigned long long", "float", "double", "string");
# unions cannot have an octet as a discriminator (in pre IDL-4) ... apparently an octet
# wasn't a discrete type or something ...
my @idltype_unidisc = ("boolean", "char", "unsigned short", "unsigned long", "unsigned long long");
my @ctype = ("bool", "uint8_t", "uint16_t", "uint32_t", "uint64_t", "float", "double", "char *");
my @cpptype = ("bool", "uint8_t", "uint16_t", "uint32_t", "uint64_t", "float", "double", "std::string");
my @probs = do {
  my @ps = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, # u0 .. u7
            0.1, # bitmask
            0.1, # enum
            0.1, # typedef
            0.7, # sequence
            0.5, # array
            2.0, # struct
            0.5  # union
           );
  disabletype(\@ps, "typedef") if $workaround_no_silly_typedefs;
  disabletype(\@ps, "uni") if $workaround_no_union;
  disabletype(\@ps, "bm") if $workaround_no_bitmask;
  normprobs @ps;
};

my @extensibility = ('@final', '@appendable', '@mutable');

# union_case ::= type_spec declarator
#   declarator ::= simple | complex
#   complex ::= array_declarator
# so IDLC is supposed to handle arrays in union cases correctly, but it doesn't and thus
# we explicitly disable the generation of them if the corresponding workaround is enabled
my @unicaseprobs = do {
  my @ps = @probs;
  disabletype(\@ps, "bm") if $workaround_no_bitmask_in_union;
  disabletype(\@ps, "enu") if $workaround_no_enum_in_union;
  disabletype(\@ps, "ary") if $workaround_no_arrays_in_union;
  normprobs @ps;
};

my @corpus;
if (defined $corpus_file && open FH, "< $corpus_file") {
  print "reading corpus ...\n";
  while (<FH>) { push @corpus, eval $_ };
  close FH;
  printf "... read %d types\n", scalar @corpus;
}
my ($nextident, $vbglobal) = do { # can add a unique prefix
  my $s = "";
  #my $n = @corpus;
  #while ($n != 0) {
  #  print ".";
  #  $s .= chr(ord('A') + ($n % 26));
  #  $n = int($n / 26);
  #}
  $s = reverse $s;
  ("a${s}00000", "${s}00000");
};

open IDL, ">xxx-types.idl" or die "can't open xxx-types.idl";
print IDL <<EOT;
/* seed: $seed */
module x {
EOT
;
open CYC, ">xxx.cpp" or die "can't open xxx.cpp";
print CYC <<EOT;
// seed: $seed
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <chrono>
#include <inttypes.h>

#include "dds/dds.h"
#include "dds/dds.hpp"

#include "xxx-types.h"
#include "xxx-types.hpp"

using namespace std::chrono_literals;
using namespace org::eclipse::cyclonedds;

static dds_qos_t *cqos;
static const std::vector<uint8_t> cpp_userdata = {'C','+','+'};
static const auto repr_cpp = dds::core::policy::DataRepresentationId::XCDR2;
static auto cppqosR = dds::sub::qos::DataReaderQos()
  << dds::core::policy::Reliability::Reliable()
  << dds::core::policy::UserData(cpp_userdata);
static auto cppqosW = dds::pub::qos::DataWriterQos()
  << dds::core::policy::Reliability::Reliable()
  << dds::core::policy::UserData(cpp_userdata);

static int collectmatches(dds_entity_t es[])
{
  int ret = 0;
  for (int i = 0; i < 4; i++) {
    if (i < 2) {
      dds_publication_matched_status_t pm;
      dds_get_publication_matched_status(es[i], &pm);
      ret |= (pm.current_count == 2) << i;
    } else {
      dds_subscription_matched_status_t sm;
      dds_get_subscription_matched_status(es[i], &sm);
      ret |= (sm.current_count == 2) << i;
    }
  }
  return ret;
}

static bool waitformatches(dds_entity_t wrcpp, dds_entity_t rdcpp, dds_entity_t wrc, dds_entity_t rdc)
{
  dds_entity_t es[] = { wrcpp, wrc, rdcpp, rdc };
  dds_entity_t ws = dds_create_waitset(DDS_CYCLONEDDS_HANDLE);
  if (ws < 0) abort();
  for (int i = 0; i < 4; i++) {
    dds_set_status_mask(es[i], (i < 2) ? DDS_PUBLICATION_MATCHED_STATUS : DDS_SUBSCRIPTION_MATCHED_STATUS);
    dds_waitset_attach(ws, es[i], 0);
  }
  dds_time_t abstimeout = dds_time() + DDS_SECS(5);
  int c;
  while ((c = collectmatches(es)) != 0xf) {
    for (int i = 0; i < 4; i++)
      if (c & (1 << i))
        (void)dds_waitset_detach(ws, es[i]);
    if (dds_waitset_wait_until(ws, NULL, 0, abstimeout) <= 0)
      break;
  }
  (void)dds_delete(ws);
  return collectmatches(es) == 0xf;
}

static int cmpfail(void) { return 0; }
EOT
;

# add a bit of junk to the corpus
#push @corpus, genstr(0) for (1 .. 100);

my @generated_types = ();
my @typenames = ();
my @inheritable_types = ();
my $identifiers_consumed = 0;
for (1 .. $ntypes) {
  my $floatpct = 100.0 * $_ / $ntypes;
  @inheritable_types = @generated_types;
  # avoid generating the same type twice (O(n^2) is not a problem yet)
  (my $ident_start = $nextident) =~ s/^a0*(\d+)$/$1/;
 RETRY:
  my $t = genstr(0);
  for (@corpus) {
    goto RETRY if eqtype($t, $_);
  }
  (my $ident_end = $nextident) =~ s/^a0*(\d+)$/$1/;
  $identifiers_consumed += $ident_end - $ident_start;
  push @corpus, $t;
  push @generated_types, $t;
  push @typenames, $t->[1];
  my $idl = genidltd($t);
  print IDL $idl;
  (my $idlstr = $idl) =~ s,^(.*)$,"$1\\n",mg;
  print CYC "const std::string idl_$t->[1] = $idlstr;\n";
  my $cmpc = gencmp_c($t);
  print CYC $cmpc;
  my $cmpcpp = gencmp_cpp($t);
  print CYC $cmpcpp;
  my $test = gentest($floatpct, $t);
  print CYC $test;
}

print CYC <<EOT;

int main(int argc, char **argv)
{
  std::string cfg0 = "";
  if (const char *cyc_uri_raw = std::getenv("CYCLONEDDS_URI"))
    cfg0 = std::string(cyc_uri_raw);
  const std::string cfgK = cfg0 + ",<Disc><ExternalDom>0</></>";
  const dds::domain::qos::DomainParticipantQos dpqos;
  dds::domain::DomainParticipant dpcppW(0, dpqos, nullptr, dds::core::status::StatusMask(), cfgK);
  dds::domain::DomainParticipant dpcppR(1, dpqos, nullptr, dds::core::status::StatusMask(), cfgK);
  dds::domain::DomainParticipant dpcWwrap(3, dpqos, nullptr, dds::core::status::StatusMask(), cfgK);
  dds::domain::DomainParticipant dpcRwrap(4, dpqos, nullptr, dds::core::status::StatusMask(), cfgK);
  const dds_entity_t dpcW = dpcWwrap->get_ddsc_entity();
  const dds_entity_t dpcR = dpcRwrap->get_ddsc_entity();

  cqos = dds_create_qos ();
  dds_qset_reliability (cqos, DDS_RELIABILITY_RELIABLE, DDS_INFINITY);
  dds_qset_userdata (cqos, "C", 1);
  if (argc > 1 && atoi(argv[1])) {
    dds_data_representation_id_t repr_c[] = { DDS_DATA_REPRESENTATION_XCDR2 };
    dds_qset_data_representation (cqos, 1, repr_c);
    cppqosR << dds::core::policy::DataRepresentation({repr_cpp});
    cppqosW << dds::core::policy::DataRepresentation({repr_cpp});
  }
EOT
for (@typenames) {
  print CYC "  test_$_(dpcppW, dpcppR, dpcW, dpcR);\n";
}
;
print CYC <<EOT;
  dds_delete_qos (cqos);
  return 0;
}
EOT
;
close CYC;
print IDL "};\n";
close IDL;

if (defined $corpus_file && open FH, ">> $corpus_file") {
  $Data::Dumper::Indent = 0;
  $Data::Dumper::Useqq = 0;
  for (@generated_types) {
    print FH Dumper($_), "\n";
  }
  close FH;
}
print "identifiers consumed: $identifiers_consumed\n";

sub fmtdiscval {
  my ($t, $v) = @_;
  if ($t->[2] eq "u0") {
    return $v ? "true" : "false";
  } elsif ($t->[2] eq "u1") {
    return "'".chr ($v + ord ("A"))."'";
  } else {
    return $v;
  }
}

sub gencmp_c {
  my ($t) = @_;
  my $res = "";
  $res .= "static int cmp_c_$t->[1] (const x_$t->[1] *a, const x_$t->[1] *b) {\n";
  $res .= gencmp_c1 ("  ", $t, "");
  $res .= "  return 1;\n}\n\n";
  return $res;
}

sub gencmp_c1 {
  my ($ind, $t, $path) = @_;
  if ($t->[0] eq "typedef") {
    return gencmp_c1 ($ind, $t->[2], $path);
  } elsif ($t->[0] =~ /^u([0-6])$/ || $t->[0] =~ /^(?:bm|enu)$/) {
    return "${ind}if (a->$path != b->$path) return cmpfail();\n";
  } elsif ($t->[0] eq "u7") {
    return "${ind}if (strcmp (a->$path, b->$path) != 0) return cmpfail();\n";
  } elsif ($t->[0] eq "seq") {
    my $idx = "i".length $path;
    return ("${ind}if (a->$path._length != b->$path._length) return cmpfail();\n" .
            "${ind}for (uint32_t $idx = 0; $idx < a->$path._length; $idx++) {\n" .
            gencmp_c1 ("${ind}  ", $t->[2], "$path._buffer[$idx]") .
            "${ind}}\n");
  } elsif ($t->[0] eq "ary") {
    my $len = $t->[3]; die unless $len > 0;
    my $idx = "i".length $path;
    return ("${ind}for (uint32_t $idx = 0; $idx < $len; $idx++) {\n" .
            gencmp_c1 ("${ind}  ", $t->[2], "$path\[$idx]") .
            "${ind}}\n");
  } elsif ($t->[0] eq "str") {
    my $sep = length $path == 0 ? "" : ".";
    my $tmp = "";
    if (defined $t->[3]) {
      $tmp .= gencmp_c1 ("${ind}", $t->[3], "$path${sep}parent");
    }
    for (my $i = 4; $i < @$t; $i++) {
      my ($name, $st, $optflag) = @{$t->[$i]};
      if (not $optflag) {
        $tmp .= gencmp_c1 ("${ind}", $st, "$path${sep}$name");
      } else {
        $tmp .= "${ind}if ((a->$path${sep}$name==0) != (b->$path${sep}$name==0)) return cmpfail();\n";
        $tmp .= "${ind}if (a->$path${sep}$name) {\n";
        if (isstring($st)) {
          $tmp .= gencmp_c1 ("${ind}  ", $st, "$path${sep}$name");
        } else {
          # need to dereference suffixing with [0] is an alternative way of doing that
          $tmp .= gencmp_c1 ("${ind}  ", $st, "$path${sep}$name\[0]");
        }
        $tmp .= "${ind}}\n";
      }
    }
    return $tmp;
  } elsif ($t->[0] eq "uni") { # uni name disctype hasdef case...
    my $tmp = "${ind}if (a->$path._d != b->$path._d) return cmpfail();\n";
    my $hasdef = $t->[3];
    $tmp .= "${ind}switch (a->$path._d) {\n";
    for (my $i = 4; $i < @$t; $i++) {
      next if $t->[$i] eq "skip";
      my $discval = fmtdiscval ($t, $i - 4);
      $tmp .= ($i == $#{$t} && $hasdef) ? "${ind}  default:\n" : "${ind}  case $discval:\n";
      if (ref $t->[$i] eq "ARRAY") {
        my ($name, $st) = @{$t->[$i]};
        $tmp .= gencmp_c1 ("${ind}    ", $st, "$path._u.$name");
        $tmp .= "${ind}    break;\n";
      }
    }
    $tmp .= "${ind}}\n";
    return $tmp;
  } else {
    die;
  }
}

sub gencmp_cpp {
  my ($t) = @_;
  my $res = "";
  $res .= "static int cmp_cpp_$t->[1] (const x::$t->[1]& a, const x::$t->[1]& b) {\n";
  $res .= gencmp_cpp1 ("  ", $t, "", "");
  $res .= "  return 1;\n}\n\n";
  return $res;
}

sub gencmp_cpp1 {
  my ($ind, $t, $path, $op) = @_;
  if ($t->[0] eq "typedef") {
    return gencmp_cpp1 ($ind, $t->[2], $path, $op);
  } elsif ($t->[0] =~ /^u([0-7])$/ || $t->[0] =~ /^(?:bm|enu)$/) {
    return "${ind}if (a.$path$op != b.$path$op) return cmpfail();\n";
  } elsif ($t->[0] eq "seq") {
    my $idx = "i".length $path;
    return ("${ind}if (a.$path$op.size() != b.$path$op.size()) return cmpfail();\n" .
            "${ind}for (uint32_t $idx = 0; $idx < a.$path$op.size(); $idx++) {\n" .
            gencmp_cpp1 ("${ind}  ", $t->[2], "$path$op\[$idx]", "") .
            "${ind}}\n");
  } elsif ($t->[0] eq "ary") {
    my $len = $t->[3]; die unless $len > 0;
    my $idx = "i".length $path;
    return ("${ind}for (uint32_t $idx = 0; $idx < $len; $idx++) {\n" .
            gencmp_cpp1 ("${ind}  ", $t->[2], "$path$op\[$idx]", "") .
            "${ind}}\n");
  } elsif ($t->[0] eq "str") {
    my $sep = length $path == 0 ? "" : ".";
    my $tmp = "";
    if (defined $t->[3]) {
      $tmp .= gencmp_cpp1 ("${ind}", $t->[3], $path, $op);
    }
    for (my $i = 4; $i < @$t; $i++) {
      my ($name, $st, $optflag) = @{$t->[$i]};
      if (not $optflag) {
        $tmp .= gencmp_cpp1 ("${ind}", $st, "$path$op$sep$name", "()");
      } else {
        $tmp .= "${ind}if (a.$path$op$sep$name().has_value() != b.$path$op$sep$name().has_value()) return cmpfail();\n";
        $tmp .= "${ind}if (a.$path$op$sep$name().has_value()) {\n";
        $tmp .= gencmp_cpp1 ("${ind}  ", $st, "$path$op$sep$name().value", "()");
        $tmp .= "${ind}}\n";
      }
    }
    return $tmp;
  } elsif ($t->[0] eq "uni") { # uni name disctype hasdef case...
    my $tmp = "${ind}if (a.$path$op._d() != b.$path$op._d()) return cmpfail();\n";
    my $hasdef = $t->[3];
    $tmp .= "${ind}switch (a.$path$op._d()) {\n";
    for (my $i = 4; $i < @$t; $i++) {
      next if $t->[$i] eq "skip";
      my $discval = fmtdiscval ($t, $i - 4);
      $tmp .= ($i == $#{$t} && $hasdef) ? "${ind}  default:\n" : "${ind}  case $discval:\n";
      if (ref $t->[$i] eq "ARRAY") {
        my ($name, $st) = @{$t->[$i]};
        $tmp .= gencmp_cpp1 ("${ind}    ", $st, "$path$op.$name", "()");
        $tmp .= "${ind}    break;\n";
      }
    }
    $tmp .= "${ind}}\n";
    return $tmp;
  } else {
    die;
  }
}

sub gentest {
  my ($floatpct, $t) = @_;
  my $res = "";
  # Test values
  my @vs = ();
  for (1 .. $nvalues) {
    my @v = geninit ($t);
    push @vs, \@v;
  }
  $res .= "static void test_$t->[1] (dds::domain::DomainParticipant& dpcppW, dds::domain::DomainParticipant& dpcppR, dds_entity_t dpcW, dds_entity_t dpcR) {\n";
  $res .= "  int pct = $floatpct;\n";
  $res .= "  printf(\"\%d\%\%\\r\", (int)(pct + 0.5)); fflush(stdout);\n";
  for (@vs) {
    $res .= "  $_" for (@{$_->[0]});
  }
  $res .= "  const x_$t->[1] vs_c[] = {\n";
  $res .= "    $_->[1],\n" for (@vs);
  $res .= "  };\n";
  for (@vs) {
    $res .= "  $_" for (@{$_->[2]});
  }
  $res .= "  std::vector<x::$t->[1]> vs_cpp;\n";
  for (@vs) {
    $res .= "    vs_cpp.push_back($_->[3]);\n";
  }
  $res .= <<EOT;
  dds::topic::Topic<x::$t->[1]> tpcppW(dpcppW, \"$t->[1]\");
  dds::topic::Topic<x::$t->[1]> tpcppR(dpcppR, \"$t->[1]\");
  dds::pub::Publisher pubcpp(dpcppW);
  dds::pub::DataWriter<x::$t->[1]> wrcpp(pubcpp, tpcppW, cppqosW);
  dds::sub::Subscriber subcpp(dpcppR);
  dds::sub::DataReader<x::$t->[1]> rdcpp(subcpp, tpcppR, cppqosR);
  dds_entity_t tpcW = dds_create_topic(dpcW, &x_$t->[1]_desc, \"$t->[1]\", nullptr, nullptr);
  if (tpcW < 0) abort();
  dds_entity_t tpcR = dds_create_topic(dpcR, &x_$t->[1]_desc, \"$t->[1]\", nullptr, nullptr);
  if (tpcR < 0) abort();
  dds_entity_t wrc = dds_create_writer(dpcW, tpcW, cqos, nullptr);
  if (wrc < 0) abort();
  dds_entity_t rdc = dds_create_reader(dpcR, tpcR, cqos, nullptr);
  if (rdc < 0) abort();
  if (!waitformatches(wrcpp->get_ddsc_entity(), rdcpp->get_ddsc_entity(), wrc, rdc)) {
    std::cout << "timed out waiting for match on:" << std::endl << idl_$t->[1];
    abort ();
  }
  dds_set_status_mask(rdcpp->get_ddsc_entity(), DDS_DATA_AVAILABLE_STATUS);
  dds_set_status_mask(rdc, DDS_DATA_AVAILABLE_STATUS);
  dds_entity_t ws = dds_create_waitset(DDS_CYCLONEDDS_HANDLE);
  for (size_t k = 0; k < 2 * vs_cpp.size(); k++) {
    if (dds_waitset_attach(ws, rdcpp->get_ddsc_entity(), 0) < 0) abort();
    if (dds_waitset_attach(ws, rdc, 1) < 0) abort();
    const size_t i = k % vs_cpp.size();
    if (k < vs_cpp.size()) {
      wrcpp.write(vs_cpp[i]);
    } else {
      if (dds_write(wrc, &vs_c[i]) < 0) abort();
    }
    const std::string lang = (k < vs_cpp.size()) ? "C++" : "C";
    uint32_t status[] = { 0, 0 };
    dds_time_t abstimeout = dds_time() + DDS_SECS(5);
    while (1) {
      if (!status[0] && dds_take_status(rdcpp->get_ddsc_entity(), &status[0], DDS_DATA_AVAILABLE_STATUS) < 0) abort ();
      if (status[0]) dds_waitset_detach(ws, rdcpp->get_ddsc_entity());
      if (!status[1] && dds_take_status(rdc, &status[1], DDS_DATA_AVAILABLE_STATUS) < 0) abort ();
      if (status[1]) dds_waitset_detach(ws, rdc);
      if (status[0] && status[1])
        break;
      if (dds_waitset_wait_until (ws, nullptr, 0, abstimeout) <= 0) {
        std::cout << "timed out waiting for " << lang << " data " << i << " on:" << std::endl << idl_$t->[1];
        abort();
      }
    }
    dds::sub::LoanedSamples<x::$t->[1]> xcpp = rdcpp.take();
    if (xcpp.length() != 1) abort();
    dds::sub::LoanedSamples<x::$t->[1]>::const_iterator iter = xcpp.begin();
    if (!cmp_cpp_$t->[1] (vs_cpp[i], iter->data())) {
      std::cout << "C++ cmp failed for " << lang << " data " << i << " on:" << std::endl << idl_$t->[1];
      abort();
    }
    void *xcraw = 0;
    dds_sample_info_t si;
    if (dds_take(rdc, &xcraw, &si, 1, 1) != 1) abort ();
    if (!cmp_c_$t->[1] (&vs_c[i], (x_$t->[1] *)xcraw)) {
      std::cout << "C cmp failed for " << lang << " data " << i << " on:" << std::endl << idl_$t->[1];
      abort();
    }
    if (dds_return_loan(rdc, &xcraw, 1) < 0) abort();
  }
  if (dds_delete(ws) < 0) abort();
  if (dds_delete(rdc) < 0) abort();
  if (dds_delete(wrc) < 0) abort();
  if (dds_delete(tpcR) < 0) abort();
  if (dds_delete(tpcW) < 0) abort();
}
EOT
    ;
  return $res;
}

sub geninit {
  my ($t) = @_;
  my @outc;
  my @outcpp;
  my @rs = ();
  my $t1 = geninit1c ("  ", \@outc, $t, "$vbglobal", sub { my $v = rand($_[0]); push @rs, $v; return $v });
  my $t2 = geninit1cpp ("  ", \@outcpp, $t, "$vbglobal", sub { die unless @rs; shift @rs }, undef);
  die if @rs;
  $vbglobal++;
  return (\@outc, $t1, \@outcpp, $t2);
}

sub getrand_intconst {
  my ($typecode, $rand) = @_;
  if ($typecode == 0) {
    return int(&$rand(2));
  } elsif ($typecode == 1) {
    return int(&$rand(255));
  } elsif ($typecode == 2) {
    return int(&$rand(65535));
  } elsif ($typecode == 3) {
    return int(&$rand(2147483647));
  } elsif ($typecode == 4) {
    no warnings "portable";
    my $i1 = int(&$rand(2147483647));
    my $i2 = int(&$rand(2147483647));
    return hex(sprintf "0x%08x%08x", $i1, $i2);
  } else {
    die;
  }
}

sub isstring {
  my ($t) = @_;
  if ($t->[0] eq "u7") {
    return 1;
  } elsif ($t->[0] eq "typedef") {
    return isstring ($t->[2]);
  } else {
    return 0;
  }
}

sub geninit1c {
  my ($ind, $out, $t, $idxsuf, $rand) = @_;
  if ($t->[0] eq "typedef") {
    return geninit1c ($ind, $out, $t->[2], $idxsuf, $rand);
  } elsif ($t->[0] =~ /^u([0-4])$/) {
    return getrand_intconst($1, $rand);
  } elsif ($t->[0] =~ /^u[56]$/) {
    return &$rand(1.0);
  } elsif ($t->[0] eq "u7") {
    return "(char *)\"".("x"x(int(&$rand(8))))."\"";
  } elsif ($t->[0] eq "seq") {
    my $bound = $t->[3] ? $t->[3] : 10;
    my $len = int(&$rand($bound));
    my $buf = "vb$t->[1]_$idxsuf";
    my $ctype = ($t->[2]->[0] =~ /^u(\d+)$/) ? $ctype[$1] : "x_$t->[2]->[1]";
    if ($len == 0) {
      return ("{0,0,0,0}");
    } else {
      my $tmp = "$ctype $buf\[\] = {\n$ind  ";
      for (1..$len) {
        $tmp .= geninit1c ("$ind  ", $out, $t->[2], "${idxsuf}_$_", $rand);
        $tmp .= "," if $_ < $len;
      }
      $tmp .= "}";
      push @$out, "$tmp;\n";
      return "{$len,$len,$buf,0}";
    }
  } elsif ($t->[0] eq "ary") {
    my $len = $t->[3];
    die unless $len > 0;
    my $ctype = ($t->[2]->[0] =~ /^u(\d+)$/) ? $ctype[$1] : "x_$t->[2]->[1]";
    my $tmp = "{\n$ind  ";
    for (1..$len) {
      $tmp .= geninit1c ("$ind  ", $out, $t->[2], "${idxsuf}_$_", $rand);
      $tmp .= "," if $_ < $len;
    }
    $tmp .= "}";
    return $tmp;
  } elsif ($t->[0] eq "str") {
    my $tmp = "{";
    if (defined $t->[3]) {
      $tmp .= geninit1c ("", $out, $t->[3], "${idxsuf}_", $rand);
      $tmp .= ",";
    }
    for (my $i = 4; $i < @$t; $i++) {
      my ($name, $st, $optflag) = @{$t->[$i]};
      if (not $optflag) {
        $tmp .= geninit1c ("", $out, $st, "${idxsuf}_", $rand);
      } elsif (&$rand(1.0) < 0.5) {
        $tmp .= "NULL";
      } elsif (isstring($st)) {
        $tmp .= geninit1c ("", $out, $st, "${idxsuf}_", $rand);
      } else {
        my $buf = "vbopt$t->[1]_${idxsuf}_" . nextident();
        my $ctype = ($st->[0] =~ /^u(\d+)$/) ? $ctype[$1] : "x_$st->[1]";
        my $tmp1 = "$ctype $buf = ";
        $tmp1 .= geninit1c ("$ind  ", $out, $st, "${idxsuf}_$_", $rand);
        push @$out, "$tmp1;\n";
        $tmp .= "&$buf";
      }
      $tmp .= "," if $i + 1 < @$t;
    }
    $tmp .= "}";
    return $tmp;
  } elsif ($t->[0] eq "uni") { # uni name disctype hasdef case...
    my $discval = makediscval($t, $rand);
    my ($fdiscval, $name, $st) = getactcase($t, $discval);
    ($name, $st) = getdummycase($t) if not defined $st;
    my $tmp = "{$fdiscval,{.$name=";
    $tmp .= geninit1c ("$ind  ", $out, $st, "${idxsuf}_", $rand);
    $tmp .= "}}";
    return $tmp;
  } elsif ($t->[0] eq "bm") {
    my %bitbound_map = (8 => 1, 16 => 2, 32 => 3, 64 => 4);
    my $mask = 0;
    for (my $i = 3; $i < @$t; $i++) {
      $mask |= 1 << $t->[$i]->[1];
    }
    return getrand_intconst($bitbound_map{$t->[2]}, $rand) & $mask;
  } elsif ($t->[0] eq "enu") {
    my $idx = 3 + int(&$rand(@$t - 3));
    return "x_$t->[$idx]->[0]";
  } else {
    die;
  }
}

sub geninit1cpp {
  my ($ind, $out, $t, $idxsuf, $rand, $inherit) = @_;
  if ($t->[0] eq "typedef") {
    return geninit1cpp ($ind, $out, $t->[2], $idxsuf, $rand, $inherit);
  } elsif ($t->[0] =~ /^u([0-4])$/) {
    return getrand_intconst($1, $rand);
  } elsif ($t->[0] =~ /^u[56]$/) {
    return &$rand(1.0);
  } elsif ($t->[0] eq "u7") {
    return "\"".("x"x(int(&$rand(8))))."\"";
  } elsif ($t->[0] eq "seq") {
    my $bound = $t->[3] ? $t->[3] : 10;
    my $len = int(&$rand($bound));
    my $buf = "wb$t->[1]_$idxsuf";
    my $cpptype = ($t->[2]->[0] =~ /^u(\d+)$/) ? $cpptype[$1] : "x::$t->[2]->[1]";
    push @$out, "std::vector<$cpptype> $buf;\n";
    for (1..$len) {
      my $tmp = geninit1cpp ("$ind  ", $out, $t->[2], "${idxsuf}_$_", $rand, $inherit);
      push @$out, "  $buf.push_back($tmp);\n";
    }
    return $buf;
  } elsif ($t->[0] eq "ary") {
    my $len = $t->[3];
    die unless $len > 0;
    my $buf = "wb$t->[1]_$idxsuf";
    my $cpptype = ($t->[2]->[0] =~ /^u(\d+)$/) ? $cpptype[$1] : "x::$t->[2]->[1]";
    push @$out, "std::array<$cpptype,$len> $buf;\n";
    for (1..$len) {
      my $idx = $_ - 1;
      my $tmp = geninit1cpp ("$ind  ", $out, $t->[2], "${idxsuf}_$_", $rand, $inherit);
      push @$out, "  $buf\[$idx] = $tmp;\n";
    }
    return $buf;
  } elsif ($t->[0] eq "str") {
    my $buf = "wb$t->[1]_$idxsuf";
    if (defined $t->[3]) {
      geninit1cpp ("$ind", $out, $t->[3], ${idxsuf}, $rand, defined $inherit ? $inherit : $t->[1]);
    } elsif (defined $inherit) {
      push @$out, "x::$inherit $buf;\n";
    } else {
      push @$out, "x::$t->[1] $buf;\n";
    }
    for (my $i = 4; $i < @$t; $i++) {
      my ($name, $st, $optflag) = @{$t->[$i]};
      if (not $optflag) {
        my $tmp = geninit1cpp ("$ind  ", $out, $st, "${idxsuf}_", $rand, $inherit);
        push @$out, "$buf.$name($tmp);\n";
      } elsif (&$rand(1.0) < 0.5) {
        # not set -- I suppose that's the default constructor
      } else {
        my $tmp = geninit1cpp ("$ind  ", $out, $st, "${idxsuf}_", $rand, $inherit);
        push @$out, "$buf.$name($tmp);\n";
      }
    }
    return $buf;
  } elsif ($t->[0] eq "uni") { # uni name disctype hasdef case...
    my $buf = "wb$t->[1]_$idxsuf";
    push @$out, "x::$t->[1] $buf;\n";
    my $discval = makediscval($t, $rand);
    my ($fdiscval, $name, $st) = getactcase($t, $discval);
    if (defined $st) {
      my $tmp = geninit1cpp ("$ind  ", $out, $st, "${idxsuf}_", $rand, $inherit);
      push @$out, "$buf.$name($tmp);\n";
      push @$out, "$buf._d($fdiscval);\n";
    } else {
      my ($actlabels, $maxlabels) = getnlabels($t);
      if ($actlabels < $maxlabels) {
        if ($actlabels < $maxlabels - 1) {
          push @$out, "$buf._default($fdiscval);\n";
        } else {
          push @$out, "$buf._default();\n";
        }
      }
      # keep RNGs in sync between C and C++ generators
      my @dummy;
      my ($name, $st) = getdummycase($t);
      geninit1cpp ("$ind  ", \@dummy, $st, "${idxsuf}_", $rand, $inherit);
    }
    return $buf;
  } elsif ($t->[0] eq "bm") {
    my %bitbound_map = (8 => 1, 16 => 2, 32 => 3, 64 => 4);
    my $mask = 0;
    for (my $i = 3; $i < @$t; $i++) {
      $mask |= 1 << $t->[$i]->[1];
    }
    return getrand_intconst($bitbound_map{$t->[2]}, $rand) & $mask;
  } elsif ($t->[0] eq "enu") {
    my $idx = 3 + int(&$rand(@$t - 3));
    return "x::$t->[1]::$t->[$idx]->[0]";
  } else {
    die;
  }
}

sub makediscval {
  my ($t, $rand) = @_;
  my ($act, $max) = getnlabels($t);
  if ($act == $max || $max < @$t - 2) {
    # if all values covered or we're dealing with such a restricted range that our regular
    # random number generation would yield invalid values (i.e., booleans), be sure to
    # stay in range
    return int(&$rand($max));
  } else {
    # -2 so we generate different values outside label range as well
    return int(&$rand(@$t - 2));
  }
}

sub getactcase {
  my ($t, $discval) = @_;
  my $fdiscval = fmtdiscval($t, $discval);
  my $case = 4 + $discval;
  if ($case >= @$t || $t->[$case] eq "skip") {
    return !$t->[3] ? ($fdiscval) : ($fdiscval, @{$t->[$#{$t}]});
  } else {
    $case = @$t - 1 if $case >= @$t;
    die unless ref $t->[$#{$t}] eq "ARRAY";
    $case++ while ref $t->[$case] ne "ARRAY";
    return ($fdiscval, @{$t->[$case]});
  }
}

sub getdummycase {
  my ($t) = @_;
  my $case = 4;
  die unless ref $t->[$#{$t}] eq "ARRAY";
  $case++ while ref $t->[$case] ne "ARRAY";
  return @{$t->[$case]};
}

sub getnlabels {
  my ($t) = @_;
  die if @$t >= 100; # easy way out: assume there are < 100-odd cases for all types except bools
  my $maxlabels = ($t->[2] eq "u0") ? 2 : 100;
  my $actlabels = 0;
  for (my $i = 4; $i < @$t - $t->[3]; $i++) {
    $actlabels++ if $t->[$i] ne "skip";
  }
  return ($actlabels, $maxlabels);
}

sub genidltd {
  my ($t) = @_;
  my @out = ();
  my $res = genidl1td ("", \@out, $t);
  my $datarep = "";
  $datarep = '@data_representation(XCDR2) ' if $force_xcdr2_annot;
  return (join "", @out) . $datarep . '@topic ' . $res;
};

sub genidl1 {
  my ($ind, $out, $name, $t) = @_;
  my $res = "";
  if ($t->[0] =~ /^u(\d+)$/) {
    $res = "${ind}$idltype[$1] $name;\n";
  } elsif ($t->[0] eq "seq") {
    push @$out, genidl1td ("", $out, $t);
    $res = "${ind}$t->[1] $name;\n";
  } elsif ($t->[0] eq "ary") {
    if ($t->[2]->[0] =~ /^u(\d+)$/) {
      $res = "${ind}$idltype[$1] ${name}[$t->[3]];\n";
    } else {
      push @$out, genidl1td ("", $out, $t->[2]);
      $res = "${ind}$t->[2]->[1] ${name}[$t->[3]];\n";
    }
  } else {
    push @$out, genidl1td ("", $out, $t);
    $res = "${ind}$t->[1] $name;\n";
  }
  return $res;
}

sub genidl1td {
  my ($ind, $out, $t) = @_;
  if ($t->[0] eq "typedef") {
    return ("typedef ".genidl1 ($ind, $out, $t->[1], $t->[2]));
  } elsif ($t->[0] eq "seq") {
    my $boundstr = $t->[3] ? ", $t->[3]" : "";
    if ($t->[2]->[0] =~ /^u(\d+)$/) {
      return "${ind}typedef sequence<$idltype[$1]$boundstr> $t->[1];\n";
    } else {
      push @$out, genidl1td ("", $out, $t->[2]);
      return "${ind}typedef sequence<$t->[2]->[1]$boundstr> $t->[1];\n";
    }
  } elsif ($t->[0] eq "ary") {
    if ($t->[2]->[0] =~ /^u(\d+)$/) {
      return "${ind}typedef ${idltype[$1]} $t->[1]"."[$t->[3]];\n";
    } else {
      push @$out, genidl1td ("", $out, $t->[2]);
      return "${ind}typedef $t->[2]->[1] $t->[1]"."[$t->[3]];\n";
    }
  } elsif ($t->[0] eq "str") {
    my $annots = join ' ', @{$t->[2]};
    $annots .= ' ' if $annots ne '';
    my $res = "${ind}${annots}struct $t->[1]";
    if (defined($t->[3])) {
      $res .= " : $t->[3]->[1]";
    }
    $res .= " {\n";
    for (my $i = 4; $i < @$t; $i++) {
      my $opt = $t->[$i]->[2] ? '@optional ' : '';
      $res .= genidl1 ($ind."  $opt", $out, @{$t->[$i]});
    }
    $res .= "};\n";
    return $res;
  } elsif ($t->[0] eq "uni") {
    my $hasdef = $t->[3];
    die unless $t->[2] =~ /^u([0-3])$/;
    my $res = "${ind}\@final union $t->[1] switch ($idltype_unidisc[$1]) {\n";
    if (0) {
      $res .= "  // t:";
      for (my $i = 4; $i < @$t; $i++) {
        next if $t->[$i] eq "skip";
        my $v = fmtdiscval($t, $i - 4);
        $res .= ($i == $#{$t} && $hasdef) ? " D" : " $v";
        $res .= ' |' if $t->[$i] ne "combine";
      }
      $res .= "\n";
    }
    my @gs = ();
    { # gather labels so we can permute the order in which they are listed
      my $i = 4;
      while ($i < @$t) {
        my @g = ();
        while ($i++ < @$t) {
          my $j = $i-1;
          next if $t->[$j] eq "skip";
          push @g, $j;
          last if ref $t->[$j] eq "ARRAY";
        }
        push @gs, \@g if @g > 0;
      }
    }
    if (0) {
      $res .= "  // gs:";
      for (@gs) {
        for (@$_) { my $v = fmtdiscval($t, $_ - 4); $res .= ($_ == $#{$t} && $hasdef) ? " D" : " $v"; }
        $res .= ' |';
      }
      $res .= "\n";
    }
    die if @gs == 0;
    while (@gs) {
      my $max = @gs;
      --$max if $workaround_default_always_last && $hasdef && $max > 1;
      my $i = int(rand($max));
      my $g = splice @gs, $i, 1;
      my @g = @$g;
      die unless ref $t->[$g[$#g]];
      my ($n, $st) = @{$t->[$g[$#g]]};
      while (@g) {
        my $maxj = @g;
        --$maxj if $workaround_default_always_last && $hasdef && $maxj > 1;
        my $j = int(rand($maxj));
        my $c = splice @g, $j, 1;
        my $discval = fmtdiscval($t, $c - 4);
        $res .= ($c == $#{$t} && $hasdef) ? "$ind  default: " : "$ind  case $discval: ";
      }
      $res .= genidl1 ($ind."    ", $out, ($n, $st));
    }
    $res .= "};\n";
    return $res;
  } elsif ($t->[0] eq "bm") {
    my $res = "${ind}\@final \@bit_bound($t->[2]) bitmask $t->[1] {\n";
    for (my $i = 3; $i < @$t; $i++) {
      $res .= "  \@position($t->[$i]->[1]) $t->[$i]->[0]";
      $res .= "," if $i + 1 < @$t;
      $res .= "\n";
    }
    $res .= "};\n";
    return $res;
  } elsif ($t->[0] eq "enu") {
    my $res = "${ind}\@final \@bit_bound($t->[2]) enum $t->[1] {\n";
    for (my $i = 3; $i < @$t; $i++) {
      $res .= "  \@value($t->[$i]->[1]) $t->[$i]->[0]";
      $res .= "," if $i + 1 < @$t;
      $res .= "\n";
    }
    $res .= "};\n";
    return $res;
  } else {
    die;
  }
};

sub eqtype {
  my ($a, $b) = @_;
  # typedefs are ignored when comparing types - not sure whether that is wise
  if ($a->[0] eq "typedef") {
    return eqtype ($a->[2], $b);
  } elsif ($b->[0] eq "typedef") {
    return eqtype ($a, $b->[2]);
  } elsif ($a->[0] ne $b->[0]) {
    return 0;
  } elsif ($a->[0] eq "seq") {
    return $a->[3] == $b->[3] && eqtype($a->[2], $b->[2]);
  } elsif ($a->[0] eq "ary") {
    return $a->[3] == $b->[3] && eqtype($a->[2], $b->[2]);
  } elsif ($a->[0] eq "str") {
    return 0 unless @$a == @$b;
    return 0 unless @{$a->[2]} == @{$b->[2]}; # annotations
    for (my $i = 0; $i < @{$a->[2]}; $i++) {
      return 0 unless $a->[2]->[$i] eq $b->[2]->[$i];
    }
    return 0 unless (defined($a->[3]) ? 1 : 0) == (defined($b->[3]) ? 1 : 0);
    return 0 if (defined($a->[3])) && eqtype($a->[3], $b->[3]);
    for (my $i = 4; $i < @$a; $i++) {
      return 0 unless $a->[$i]->[2] == $b->[$i]->[2]; # optional
      return 0 unless eqtype($a->[$i]->[1], $b->[$i]->[1]);
    }
    return 1;
  } elsif ($a->[0] eq "uni") {
    return 0 unless @$a == @$b && $a->[3] == $b->[3] && $a->[2] eq $b->[2];
    for (my $i = 4; $i < @$a; $i++) {
      return 0 unless ref $a->[$i] eq ref $b->[$i];
      if (ref $a->[$i] ne "ARRAY") {
        return 0 unless $a->[$i] eq $b->[$i];
      } else {
        return 0 unless eqtype($a->[$i]->[1], $b->[$i]->[1]);
      }
    }
    return 1;
  } elsif ($a->[0] =~ /^(?:bm|enu)$/) {
    return 0 unless $a->[2] == $b->[2] && @$a == @$b;
    # consider them the same if they use the same positions/values
    my @as = @$a; splice(@as,0,3); @as = sort (map { $_->[1] } @as);
    my @bs = @$b; splice(@bs,0,3); @bs = sort (map { $_->[1] } @bs);
    for (my $i = 0; $i < @as; $i++) {
      return 0 unless $as[$i] == $bs[$i];
    }
    return 1;
  } else {
    return 1;
  }
}

sub genu0 { return ["u0"]; }
sub genu1 { return ["u1"]; }
sub genu2 { return ["u2"]; }
sub genu3 { return ["u3"]; }
sub genu4 { return ["u4"]; }
sub genu5 { return ["u5"]; }
sub genu6 { return ["u6"]; }
sub genu7 { return ["u7"]; }
sub genseq { return ["seq", nextident (), gentype ($_[0] + 1, @probs), int (rand (5))]; }
sub genary { return ["ary", nextident (), gentype ($_[0] + 1, @probs), 1 + int (rand (4))]; }
sub genstr {
  my @ts = ("str", nextident (), [$extensibility[int(rand(@extensibility))]]);
  if (0&& rand (1) < 0.5 && @inheritable_types > 0) { # FIXME mustn't inherit yet
    push @ts, (splice @inheritable_types, int(rand(@inheritable_types)), 1);
  } else {
    push @ts, undef;
  }
  my $n = 1 + int (rand (4));
  while ($n--) {
    my $fn = nextident ();
    my $st = gentype ($_[0] + 1, @probs);
    # strings break the pattern in C for initializers
    # arrays break the pattern for type names ...
    my $optflag = ($st->[0] !~ /^(u7|ary)$/) && (rand()<0.2);
    $optflag = 0 if $st->[0] eq "seq" && $st->[2]->[0] eq "u7" && $workaround_no_opt_seq_string;
    push @ts, [$fn, $st, 0 + $optflag];
  }
  return \@ts;
}
sub genuni {
  my @ts = ("uni", nextident (), "u".(int(rand(4)))); # uni name disctype hasdef case...
  my $ncases = 1 + int(rand($ts[2] eq "u0" ? 2 : 5));
  # union U switch (X) { default: T; }; appears to be valid, so no need for this:
  #   push @ts, ($ncases == 1 ? 0 : int(rand(2)));
  # just randomize it always, but with a lowish probability if it means ending up with
  # only a default case
  push @ts, int(rand($ncases == 1 ? 1.05 : 2));
  # each case is:
  #   skip    this particular value doesn't map to a case
  #   combine in which case this is combined with the next (i.e., case A: case B: T n)
  #   [ n,T ] where n is the name and T the type of the case
  # if it has a default, the default is the final entry
  # the final entry is never skip nor combine
  # there is always at least one case
 GENTYPE:
  while ($ncases > 0) {
    if ($ncases > 1 && (my $whattodo = rand(1)) < 0.2) {
      # still more cases to go, so this one need not be a "real" one
      push @ts, ($whattodo < 0.1 ? "combine" : "skip");
    } else {
      my $t = gentype($_[0] + 1, @unicaseprobs);
      if ($workaround_unique_types_in_union) {
        # Because std::variant doesn't like the same type occurring multiple times among
        # its arguments, we have to work around IDLC blindly copying the cases.
        for (my $i = 4; $i < @ts; $i++) {
          next GENTYPE if ref $ts[$i] eq "ARRAY" && eqtype($t, $ts[$i]->[1]);
        }
      }
      push @ts, [nextident (), $t];
    }
    $ncases--;
  }
  die unless @ts > 4;
  die unless ref $ts[$#ts] eq "ARRAY";
  return \@ts;
}
sub genbm {
  my $bitbound = 8 << int(rand(4)); # 8, 16, 32 or 64
  my @ts = ("bm", nextident (), $bitbound); # uni name bitbound [name,pos]...
  my $nvalues = 1 + int(rand(8));
  for (my $i = 0; $i < $nvalues; $i++) {
  RETRY:
    my $pos = int(rand($bitbound));
    for (my $j = 3; $j < @ts; $j++) { # avoid duplicates
      goto RETRY if $ts[$j]->[1] == $pos;
    }
    push @ts, [nextident (), $pos];
  }
  return \@ts;
}
sub genenu {
  my $bitbound = 8 << int(rand(3)); # 8, 16, 32
  $bitbound = 32 if $workaround_only_32bit_enums;
  my %bitbound_map = (8 => 1, 16 => 2, 32 => 3, 64 => 4);
  my @ts = ("enu", nextident (), $bitbound); # uni name bitbound [name,value]...
  my $nvalues = 1 + int(rand(8));
  for (my $i = 0; $i < $nvalues; $i++) {
  RETRY:
    my $val = getrand_intconst($bitbound_map{$bitbound}, sub { return rand($_[0]) });
    $val = $i if $workaround_no_random_enums;
    for (my $j = 3; $j < @ts; $j++) { # avoid duplicates
      goto RETRY if $ts[$j]->[1] == $val;
    }
    push @ts, [nextident (), $val];
  }
  return \@ts;
}
sub gentypedef {
  # not counting another level in the type expansion
  return ["typedef", nextident (), gentype (@_)];
}

sub gentype {
  my $t = choosetype (@_);
  my $f = "gen$t";
  my $g = \&$f; # "use strict" disallows a plain &$f(@_)
  return &$g(@_);
}

sub choosetype {
  my ($lev, @probs) = @_;
  # if recursing too deep, stick to simple types
  my $r = rand ($lev == $maxlevel ? $probs[$maxsimpletype] : $probs[$#probs]);
  my $i;
  for ($i = 0; $i < $#probs; $i++) {
    last if $r < $probs[$i];
  }
  die "$lev $maxlevel $types[$i]\n" if $lev >= $maxlevel && $types[$i] !~ /^(?:u\d+|bm|enu)$/;
  return $types[$i];
}

sub nextident {
  return $nextident++;
}
