#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

error_exit()
{
    echo "There was an error running Jupyter Notebook"
    exit 1
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# MY_DIR="$(realpath -s "$SCRIPT_DIR")"
printf "SCRIPT_DIR = %s\n" $SCRIPT_DIR
# Setup python env from generated file (generated by tools/repoman/build.py)
export CARB_APP_PATH=$SCRIPT_DIR/kit
export ISAAC_PATH=$SCRIPT_DIR
export EXP_PATH=$SCRIPT_DIR/apps
# Set the path so it sees the python3 that kit uses first.
export PATH=$SCRIPT_DIR/kit/python/bin:$PATH
source ${SCRIPT_DIR}/setup_python_env.sh
python_exe=${SCRIPT_DIR}/kit/python/bin/python3
printf "Performing setup...\n"
# install jupyter as a dependency so the env supports running notebooks
${python_exe} -m pip install --upgrade --upgrade-strategy only-if-needed jupyter
printf "Setup complete\n"
#runtime configure kernelspec based on current python exe path
kernel_dir=$(mktemp -d)
cp -r ${SCRIPT_DIR}/jupyter_kernel ${kernel_dir}
sed -i 's,AUTOMATICALLY_REPLACED,'"${python_exe}"',' ${kernel_dir}/jupyter_kernel/kernel.json

# Add the kernelspec to jupyter notebook
jupyter kernelspec install ${kernel_dir}/jupyter_kernel --name isaac_sim_python3 --user

# Remove temp dir for kernel once installed
rm -rf ${kernel_dir}

# Check if we are running in a docker container
if [ -f /.dockerenv ]; then
  # Check for vulkan in docker container
  if [[ -f "${SCRIPT_DIR}/vulkan_check.sh" ]]; then
    ${SCRIPT_DIR}/vulkan_check.sh
  fi
fi

# if the user provides the argument test as the first argument,
# we attempt to run the notebook in place via commandline and exit
if [[ $1 == test ]]; then
    shift
    jupyter nbconvert --ExecutePreprocessor.timeout=600 --ExecutePreprocessor.kernel_name="isaac_sim_python3" --to notebook --execute --output=/tmp/isaac_test.ipynb  $@ || error_exit
else
    jupyter notebook --MappingKernelManager.default_kernel_name="Isaac Sim Python 3" $@ || error_exit
fi

