#! /usr/sh
# install_sdk.sh
# author: seung hee, lee
# purpose: get 'esp-idf' repository from github

cur_path=${PWD}
if [[ "$OSTYPE" == "darwin"* ]]; then
    project_path=$(dirname $(dirname $(realpath $0)))
else 
    project_path=$(dirname $(dirname $(realpath $BASH_SOURCE)))
fi

sdk_path=~/tools  # change to your own sdk path
if ! [ -d "${sdk_path}" ]; then
  mkdir ${sdk_path}
fi

esp_idf_path=${sdk_path}/esp-idf

# for Apple silicon
if [[ "$OSTYPE" == "darwin"* ]]; then
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:"/opt/homebrew/opt/openssl@3/lib/pkgconfig"
fi

# install esp-idf
cd ${sdk_path}
git clone --recursive https://github.com/espressif/esp-idf.git esp-idf
cd ${esp_idf_path}
git fetch --all --tags
git checkout v5.2.3
git submodule update --init --recursive
bash ./install.sh

source ${esp_idf_path}/export.sh

# create symbolic link
cd ${project_path}
if ! [ -d "${project_path}/sdk" ]; then
  mkdir ${project_path}/sdk
fi
ln -s ${esp_idf_path} ${project_path}/sdk/esp-idf

cd ${cur_path}