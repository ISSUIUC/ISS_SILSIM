# ISS_SILSIM

ISS Software-In-The-Loop Simulator

Non-linear 6DOF rocket trajectory simulation utility


## Installation and Setup Instructions

This repository acts as a library of sorts, and comes bundled with Flight Software. As such,
there aren't really separate steps for installing ISS_SILSIM. However, the default method of cloning
Flight Software will not *actually* install SILSIM. To do so, you'll need to run 
`git submodule update --init --recursive` in the Flight Software directory (which should be called TARS-Software).
Then running SILSIM is as easy as running the mcu_silsim environment of platformio, instead of mcu_main
like one would normally do.

### Windows Specific Steps

Windows doesn't come with a C compiler, so we'll need a few more steps when trying to run SILSIM. (Arduino bundles a limited form of gcc
so this isn't required when compiling to the real hardware)
Platformio requires there to be some form of gcc on your Path. If you haven't already done this (if you don't know what I'm talking about, you
bundles C++ compilers so this won't have come up before). Platformio only supports GCC, so we'll need to get
probably haven't), you'll need to install MinGW. This is a special build of GCC that supports compilation
on Windows. 

To install MinGW, go to https://www.mingw-w64.org/downloads/, scroll down to MingW-W64-builds and click the Github link 
(which will take you [here](https://github.com/niXman/mingw-builds-binaries/releases)). Download any of the assets
prefixed with "x86_64", although I can confirm that `x86_64-13.1.0-release-win32-seh-ucrt-rt_v11-rev1.7z` specifically works.
Then unzip the downloaded file (you might need to install [7Zip](https://www.7-zip.org/) if you haven't already).

The result of the unzipping should be a folder named something along the lines of `x86_64-13.1.0-release-win32-seh-ucrt-rt_v11-rev`.
Inside this folder you should find a folder named `mingw64`. Copy the `mingw64` folder and put it somewhere more permanent
than your `Downloads` folder. I suggest your user directory, your desktop, or perhaps your `Program Files` directory. In any case, 
once you have placed the mingw64 folder there, navigate inside that folder to the `/bin` directory inside. Note that in 
this directory, there'll be a lot of executables. One of them will be gcc. Copy the path to the `/bin` directory. For example, 
I have placed my `mingw` directory in my user directory (which is named `magil`), so the path that I would copy is 
`C:\Users\magil\mingw64\bin`.

Now we'll have to edit your computer's Path Environment Variable so that it knows where the find the gcc executable.
The easy way to do this is to open up the Windows search bar (pressing the windows key on your keyboard is a shortcut to opening
the search bar) and type in "edit the system environment variables", and click the result which opens up the Control
Panel. This will open up the System Properties dialog box. In this box, near the bottom right, is a button named
"Environment Variables". Click it to open up the Environment Variables dialog box.

In the top box, the one which says your username above it, scroll down to the Path variable, select it, and click
"Edit...". Now click "New" on the right and paste in the path you copied up above. If everything has gone well,
the path to the `bin` directory of mingw should now be a row on your Path. Press "OK" until you are out of all the dialog 
boxes. Open up a new Command Prompt, Powershell, or Terminal instance (you'll need to close an application and reopen it
for path changes to apply, including VSCode) and type in "gcc" and press enter. If all went well, you should get
an error messages about having no input files and that compilation was terminated.

### Branch Naming Convention
Please use the following naming conventions when creating branches while developing:

Your `<branch-name>` should consist of the Trello ticket ID and a short description of the work being done. For example:

`AV-420-write-cp-location-interpolation-function`

Then use the following scheme to then organize your branches:

- `<branch-name>` for small and simple contributions pertaining to a ticket
- `user/<github-username>/<branch-name>` for individual tasks or contributions, or as a sandbox for yourself
- `feature/<branch-name>` for **new** functionality that didn't exist before
- `bug/<branch-name>` for bug fixes
- `general/<branch-name>` for overall repository organization or development pipeline tweaks
- `misc/<branch-name>` or `junk/<branch-name>` for just messing around :)


An example branch for a ticket [AV-69] look like

`AV-69-implement-engine-thrust-lookup-function` for simple one-off tasks that you collaborate on with a group

or

`user/AyberkY/AV-69-implement-runga-kutta-integrator` if you're the sole contributor

or

`feature/AV-69-create-data-logger-class` if its adding new functionality or software components

and so on..

### Code Style Guide
The repository now has a GitHub Actions instance that will automatically check for code style violations!

The Actions instance **will not** inhibit a pull-request from merging. It is merely there to _encourage_ style consistency throughout our code base.

There is also an auto formatting script that will _format your code for you_! (its beautiful, you should use it) This means that you don't have to worry about coding to meet the style yourself, as you can simply run the formatting script before you commit/push your changes.

You can run the script on Linux, Mac, or WSL like so:
```
clang-format -i **/*.cpp **/*.h
```

Things to keep in mind about code formatting:
- The code style being used is defined in `.clang-format`. It currently follows Google's C++ style guide exactly.
- Changing/tweaking the style guide is always option! If you have ideas, reach out!

