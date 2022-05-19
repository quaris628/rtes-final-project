Morse Code Tutor for STM Discovery board
RTES final project for Team -... .- -.
Github: https://github.com/quaris628/rtes-final-project

This project was made for use with the Visual Studio Code (VSC) IDE with the "stm32-for-vscode" extension installed, and used the STM32CubeMX program to generate the code specific to the STM discovery board.


VSC can be installed from here:
 https://code.visualstudio.com/

To install the "stm32-for-vscode" extension for VSC:
 - Go to "Extensions" (bulding blocks icon on left ribbon)
 - Search "stm32-for-vscode" in the marketplace
 - Select the search result with that title (there should be only one, with an icon with a black background and red-orange "ST" letters.)
 - Install the extension
 
The STM32CubeMX code generation program can be installed from here:
 https://www.st.com/en/development-tools/stm32cubemx.html#get-software
To be compatible with the VSC extension, you MUST select version 6.0.0 (as of Spring 2022)
(This program is only necessary if the project is pulled fresh from github)


How to build the project (no board necessary at this step):

Use STM32CubeMX to generate code for the project (only necessary if the project is pulled fresh from github):
 - Select "Open existing projects"
 - Select the morse.ioc file in the project
 - Wait for the program to load
 - Go to the "Project Manager" tab
 - Ensure "Toolchain/IDE" has "Makefile" selected (required for the VSC extension)
 - (If developing / making changes to the project, changes to the hardware configuration may be made at this step.)
 - Select the "Generate Code" button
 - Wait for the program to load

Use the stm32-for-vscode extension to build the project
 - Go to "STM32 for VSCode" ("ST" in box icon on left ribbon)
 - Select "Build" to compile the project
   (If there are any compilation errors, they will show up at this step)
 - If you do not have the option to build it may prompt you to install the extenstion build tools
   - If you are still having build issues you may need to delete the entries for cortex-debug.armToolchainPath and cortex-debug.openocdPath from .vscode/settings.json.
  The extension will readd these to the correct path when building
 
 
How to set up the discovery board to run the project:
 - Set up the push buttons' hardware in the same way we have done in class
 - Connect the board to the computer via usb
 (no need for UART connections, they are never used)
 - Select "Flash STM32"


Hardware Setup:
  - Connect 4 buttons to D0-D3 with a 1kOhmn pull down resistor 
      - Same configuration from class