# Add to .bashrc

        export ARCH=arm64
        export CROSS_COMPILE=/home/sazan123/jimmy/toolchain/bin/aarch64-linux-android-

# How to Build

        make jimmy_defconfig
        make -j4

# How to Clean

        make clean
        make mrproper

# Output files

        - Kernel : arch/arm64/boot/Image.gz-dtb
        - module : drivers/*/*.o

# Add commit

        git init
        git add .
        git commit -m "first commit"
        git remote add origin https://github.com/sazan123/android_kernel_huawei_jimmy.git
        git push -u origin 7.x
