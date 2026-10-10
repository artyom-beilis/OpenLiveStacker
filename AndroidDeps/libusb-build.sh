source ../config.sh || exit 1

for AA in arm64-v8a,aarch64-linux-android armeabi-v7a,armv7a-linux-androideabi  x86,i686-linux-android  x86_64,x86_64-linux-android
do
    arch=$(echo $AA | cut -d, -f1)
    target_host_base=$(echo $AA | cut -d, -f2)
    echo $arch
    echo $target_host_base
    target_host=${target_host_base}$target_api
    AROOT=$OLS_SDK/$arch


    export AR=$BINPATH/llvm-ar
    export RANLIB=$BINPATH/llvm-ranlib
    export STRIP=$BINPATH/llvm-strip
    export AS=$BINPATH/${target_host}-clang
    export CC="$BINPATH/${target_host}-clang -I$AROOT/include/libusb-1.0"
    export CXX=$BINPATH/${target_host}-clang++
    export LD=$BINPATH/${target_host}-clang
    export STRIP=$BINPATH/${target_host}-strip
    export CFLAGS="-fPIE -fPIC -I$SYSROOT/include -I$AROOT/include -D_LIBCPP_HAS_NO_OFF_T_FUNCTIONS"
    export CXXFLAGS="-fPIE -fPIC -I$SYSROOT/include -I$AROOT/include -D_LIBCPP_HAS_NO_OFF_T_FUNCTIONS"
    export LDFLAGS="-L$SYSROOT/usr/lib/$target_host_base/$target_api -L$AROOT/lib"
    export API=$target_api
export NO_SONAME=1

    echo $CFLAGS
    echo $LDFLAGS

    make clean
    make distclean
    git checkout config.h.in
    PRFX=$OLS_SDK/$arch
    ./configure --disable-udev --with-gnu-ld=yes --enable-static=no --enable-shared=yes --build=x86_64-linux-gnu --target=$target_host --host=$target_host --prefix=$PRFX --with-sysroot="$SYSROOT" || exit 1
    # Fix Wired bug
    sed -i 's/archive_cmds=""/archive_cmds="\\\$CC -shared \\\$pic_flag \\\$libobjs \\\$deplibs \\\$compiler_flags \\\$wl-soname \\\$wl\\\$soname -o \\\$lib"/' libtool
    make -j $JOBS || exit 1
    make install-strip || exit 1
done

