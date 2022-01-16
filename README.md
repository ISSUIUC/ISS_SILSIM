# ISS_SILSIM

ISS Software-In-The-Loop Simulator

Non-linear 6DOF rocket trajectory simulation utility


## Installation and Setup Instructions

First time setup

```
git clone https://github.com/ISSUIUC/ISS_SILSIM
cd ISS_SILSIM
git submodule update --init --recursive
mkdir build
cd build
cmake ..
cd ..
make -C build ISS_SILSIM
./build/ISS_SILSIM
```

Note that this will build the executable in the build directory

After initial setup running
`make -C build ISS_SILSIM`
will rebuild the project

#### MacOS Installation

There are few extra steps to get your simulator runnning on your Mac.

Install the `g++` compiler through brew:

```
brew install gcc
```

Open `~/.bash_profile` in `nano`:

```
nano ~/.bash_profile
```

Add the following lines to the beginning of the file:

```
# Setting PATH for local executables
export PATH=/usr/local/bin:$PATH
```

Add symbolic link from `g++-11` to `g++`:

```
ln -s /usr/local/bin/g++-11 /usr/local/bin/g++
```
#### Windows Installation

On Windows it is recommended to use visual studio, but gcc and clang both work

Visual Studio instructions:

Get visual studio here: https://visualstudio.microsoft.com/

When installing visual studio install the c++ desktop development package

Once visual studio finishes installing clone the repo

`git clone https://github.com/ISSUIUC/ISS_SILSIM`

In visual studio choose "Open a local folder" and open the cloned repo

After visual studio finishes loading the project go to "Select Startup Item" in the top center of the screen and select "ISS_SILSIM.exe" from the dropdown

By default visual studio will run the program in out/build/<build-type>. To make silsim run in the correct directory go to "Debug" > "Debug and launch settings for ISS_SILSIM".

Add `"currentDir": "${workspaceRoot}"` to the json under the `"name"` property in configurations

Build and run the project by clicking the run button or by pressing "F5"

#### Python Dependencies

Run the following command to include the dependencies for the Python simulator in your environment:

```
pip install -r requirements.txt
```


## Development Workflow

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

