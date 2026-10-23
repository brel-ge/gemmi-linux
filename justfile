YOCTO_ENV := ". /opt/fsl-imx-fb/gemmi-sdk-wrynose/environment-setup-cortexa9t2hf-neon-poky-linux-gnueabi && "
DOCKER_IMAGE := "gitregistry.brel.ch/24_ge-team/3451_gemmi/docker_gemmi_host:wrynose"
DOCKER_RUN := "docker run --rm -u " + `id -u` + ":" + `id -g` + " -v " + `pwd` + ":/workspace -w /workspace " + DOCKER_IMAGE
DOCKER_IT := "docker run --rm -u " + `id -u` + ":" + `id -g` + " -v " + `pwd` + ":/workspace -w /workspace -it " + DOCKER_IMAGE
MAKE := "make -j 16 HOSTCC=x86_64-fslcsdk-linux-gcc"

# Default target
default: build

docker_build:
  {{ YOCTO_ENV }} make -j 16

build:
  {{ DOCKER_RUN }} just docker_build


docker_configure_gemmi:
  {{ YOCTO_ENV }} make gemmi_defconfig 

configure_gemmi:
  {{ DOCKER_RUN }} just docker_configure_gemmi

docker_configure_imx8_var_defconfig:
  {{ YOCTO_ENV }} make imx8_var_defconfig

configure_imx8_var_defconfig:
  {{ DOCKER_RUN }} just docker_configure_imx8_var_defconfig

install:
  cp arch/arm/boot/zImage /tftpboot/zImage-gemmi
  cp arch/arm/boot/dts/digi/imx6qp-gemmi.dtb /tftpboot

docker_mrproper:
  {{ YOCTO_ENV }} make mrproper

mrproper:
  {{ DOCKER_RUN }} just docker_mrproper

docker_menuconfig:
  {{ YOCTO_ENV }} {{MAKE}} menuconfig

menuconfig:
  {{ DOCKER_IT }} just docker_menuconfig

docker_install_modules:
  rm -rf .modules-install
  {{ YOCTO_ENV }} make INSTALL_MOD_PATH=/workspace/.modules-install modules_install

install_modules:
  {{ DOCKER_RUN }} just docker_install_modules
  # depmod is missing in the docker image, run it on the host
  depmod -b .modules-install `cat include/config/kernel.release`
  rm -f .modules-install/lib/modules/*/build
  sudo rm -rf /exports/gemmi-image-dev/lib/modules/`cat include/config/kernel.release`
  sudo cp -a --no-preserve=ownership .modules-install/. /exports/gemmi-image-dev/
