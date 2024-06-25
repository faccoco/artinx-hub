# -DCMAKE_TOOLCHAIN_FILE=~/workspace/package/vcpkg/scripts/buildsystems/vcpkg.cmake

sudo apt-get install libdrm-dev libxxf86vm-dev libxt-dev xutils-dev flex bison xcb libx11-xcb-dev libxcb-glx0 libxcb-glx0-dev xorg-dev libxcb-dri2-0-dev libtool autoconf zip unzip git pip vim gnome-shell-extensions tweak curl gperf libegl1-mesa-dev nasm autoconf-archive python3-jinja2

git clone https://github.com/microsoft/vcpkg.git

./vcpkg install boost opencv4[contrib,ffmpeg] glm caf glew glfw3 opengl fmt cpp-httplib bullet3 nlohmann-json magic-enum eigen3 spdlog

# sudo apt install -y linux linux-image-3.13.0-24-generic linux-headers-3.13.0-24
