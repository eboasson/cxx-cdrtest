set -o pipefail

dir=`dirname $0`
cdds=$dir/cdds/CC/install
cxx=.
iox=$dir/iceoryx/install/include/iceoryx/v1.91.0

sosuf=.so
[ `uname -s` = Darwin ] && sosuf=.dylib

function setpaths {
    if [[ "$1" =~ ^(c|cxx|iox):(.*)$ ]] ; then
        _p="${BASH_REMATCH[2]}"
        case "${BASH_REMATCH[1]}" in
            c) cdds="$_p" ;;
            cxx) cxx="$_p" ;;
            iox) iox="$_p" ;;
        esac
    else
        echo "-P$1: should be of the form (c|cxx|iox):PATH" >&2
        exit 3
    fi
}

repeat=false
keepgoing=false
corpusfile=
args=
while getopts "ac:i:kl:n:P:r" opt ; do
    case $opt in
        k) keepgoing=true ;;
        r) repeat=true ;;
        P) setpaths $OPTARG ;;
        a) args="$args $a" ;;
        c) args="$args -$opt$OPTARG" ; corpusfile="$OPTARG" ;;
        *) args="$args -$opt$OPTARG" ;;
    esac
done
shift $((OPTIND-1))

skip_test_generation=
if [ "$1" = "rebuild" ] ; then
    skip_test_generation=":"
    seed=
fi

if [ -n "$iox" ] ; then
    if [[ "$iox" =~ '/include/' ]] ; then
        : # presumably a new one
    else
        iox="$iox/include"
    fi
fi
cxxinc=
cxx2=
if [ -n "$cxx" ] ; then
    if [ -d "$cxx/src/ddscxx" ] ; then
        cxxinc="$cxx/src/ddscxx/include"
        [ -d ../src/ddscxx/include ] && cxx2=../src/ddscxx/include
    else
        cxxinc="$cxx/include/ddscxx"
    fi
fi
rpath=
for x in $cxx $cdds $iox ; do
    rpath="$rpath -Wl,-rpath,$x/lib"
done

while true ; do
    ($skip_test_generation perl -w $dir/cxx-cdrtest.pl $args "$@" && \
        $cdds/bin/idlc -l $cxx/lib/libcycloneddsidlcxx$sosuf xxx-types.idl && \
        $cdds/bin/idlc xxx-types.idl && \
        clang -I $cdds/include -g -Wall -c xxx-types.c && \
        time clang++ -std=gnu++17 -I$cxxinc ${cxx2:+-I}$cxx2 -I$cdds/include ${iox:+-I}$iox -L$cxx/lib -L$cdds/lib $rpath -g -Wall -Wno-switch-bool xxx*.cpp xxx-types.o -o xxx -lddscxx -lddsc && \
        echo "without data_representation QoS" && ./xxx && \
        echo "with data_representation QoS = XCDR2-only" && ./xxx 1) 2>&1 | tee run.log
    exitcode=$?
    if [ $exitcode -eq 130 ] ; then
        echo "stopping on interrupt"
        exit 0
    elif [ $exitcode -ne 0 ] ; then
        seed=`gawk '{print $3;exit}' xxx-types.idl`
        zf=failure-$seed.zip
        if [ -f $zf ] ; then
            echo "$zf already exists, giving up"
            exit 1
        fi
        zip -9j $zf \
            ../../cxx-cdrtest.{bash,pl} \
            xxx.cpp \
            xxx-types.idl \
            xxx-types.[ch] \
            xxx-types.[ch]pp \
            $corpusfile \
            run.log || exit 2
        $keepgoing || break
    fi
    seed=
    $repeat || break
done
