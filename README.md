<!--
 Copyright 2023 rafae

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
-->

# Vinho diesel V4

## Third Party Libraries

-   [QuickPID](https://github.com/lbussy/QuickPID).
-   [mpu6050](https://github.com/ElectronicCats/mpu6050)

## Setup errors

### Platform io doesn't recognize the package?

If you are getting an error simmilar to `Could not find the package with 'espressif/toolchain-riscv32-esp @ 8.4.0+2021r2-patch3' requirements for your system 'windows_amd64` you can try reeinstalling Platformio, you can follow this steps:

1. Uninstall the vscode extension.
2. Delete the `.platformio` folder located at `C:\user\YOUR_USER\.platformio` on windows and `/home/YOUR_USER/.platformio` on linux.
    > You can find some info here [https://community.platformio.org/t/how-to-uninstall-platformio-cli/20417](https://community.platformio.org/t/how-to-uninstall-platformio-cli/20417)
