# How to Build

        get Toolchain From android git server , codesourcery and etc ..
            aarch64-linux-android-4.9

        edit Makefile
        edit "CROSS_COMPILE" to right toolchain path(You downloaded).
        EX) CROSS_COMPILE= $(android platform directory you download)/android/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-
        Ex) CROSS_COMPILE=/usr/local/toolchain/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-
        // check the location of toolchain
        or
        Ex) export CROSS_COMPILE=arm-linux-androideabi-
        Ex) export PATH=$PATH:<toolchain_parent_dir>/aarch64-linux-android-4.9/bin

        $ make ARCH=arm64 huawei6750_66_n_defconfig
        $ make ARCH=arm64 Image.gz-dtb

# Output files

        Kernel : arch/arm64/boot/Image.gz-dtb
        module : drivers//.o

# How to Clean 

        $ make ARCH=arm64 distclean

-----------------------------------------------------------------------------

        git init
        git add .
        git commit -m "first commit"
        git remote add origin https://github.com/sazan123/android_kernel_huawei_huawei6750_66_n.git
        git push -u origin Android-7.0

