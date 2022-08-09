# ISS_Learning_Controls
Learn the fundamentals of controls!

## Purpose

The purpose of this repository is to educate Illinois Space Society (ISS) members about controls. This repository includes resources and demos in which members can learn and apply control theory. The resources provided allow for learning the basics of controls theory. The demo is a playground to tinker with the control system, and get immediate feedback on the quality of system created. This repository contains all the demos and resources necessary to get a basic understanding of controls to be applied to numerous projects in ISS.

## Resources
#### YouTube Playlists

- [Essence of Linear Algebra](https://www.youtube.com/watch?v=kjBOesZCoqc&list=PL0-GT3co4r2y2YErbmuJw2L5tW4Ew2O5B): Matrix math fundamentals
- [Controls Bootcamp](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m): Basics of controls

#### Coding Resources

- [Codacademy Python](https://www.codecademy.com/learn/learn-python): Basics of Python
- [Hackerrank](https://www.hackerrank.com): Coding practice
- [AE 353 Controls Examples](https://github.com/tbretl/ae353-sp21/tree/main/examples): Controls practice (Python)
- [Control_Bootcamp_S_Brunton](https://github.com/bertozzijr/Control_Bootcamp_S_Brunton): Controls practice (Matlab)
## File Structure

#### Basics of Terminal Commands

Programming is extremly heavy with the command line usage, whether that be macOS, Linux, or Windows. Before moving on, review these quick tutorials based on your system. 

:warning: **If you are using Windows, please make sure to use PowerShell as your terminal!**

- [Tutorial](https://github.com/ISSUIUC/ISS_Learning_Controls.git)
- [Cheat Sheet](https://www.guru99.com/linux-commands-cheat-sheet.html)

#### What are in these files?

`controls-startracker-demo` contains the control code and simulation for a satellite from a project in AE 353. This is where your sandbox for your control code will live for this simulator. 

`controls-pendulum-demo` contains the control code and simulation for a pendulum from the Controls BootCamp from Steve Brunton. This is where your sandbox for your control code will live for this simulator. 

`requirements.txt` contains all the necessary packages required to run the control simulation. Use of this file will be further explained later. 

## Setting up your computer

#### MacOS

Install Xcode Command Line Tools:

```
$ xcode-select --install
```

Install Homebrew to install certain packages that can be useful. First, check if you have it installed:

```
$ which brew
/usr/local/bin/brew
```

If it's not installed, this should be output:
```
$ which brew
brew not found
```

To install Homebrew, run the following command:
```
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Install Git Large File Storage:
```
$ brew install git-lfs
```

#### Windows

Install Microsoft C++ Build Tools:

Pybullet on Windows requires us to install Microsoft C++ Build Tools:

Go to the page [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) and follow these steps:


* Click Download Build Tools to download a file named something like `vs_buildtool_*.exe`.
* Double-click this `.exe` file. If a security warning appears, click Run.
* A *Visual Studio Installer* window should appear. Click Continue and wait for installation to complete.
* A *Visual Studio Build Tools 2019* window should appear with various package options. Choose the Desktop development with C++ package, click Install at the bottom-right corner of the window, and wait for installation to complete.
* You should now see an *Installed* list with *Visual Studio Build Tools 2019*. If so, then you are done and can exit the installer.
* Reboot your computer if asked 


## Using GitHub

#### What is GitHub?

A more in-depth guide on GitHub can be found on [our wiki](https://wiki.illinois.edu/wiki/pages/viewpage.action?pageId=779063487)

GitHub is a repository (filesystem) management tool that allows for version control and management amongst numerous people. Learning how to use Git is extremely useful, and I highly reccommend learning how to use Git to be successful in developing any complex programs.

Using GitHub, you can download the files from the internet and make changes locally. Then once you make the changes you're satisfied with, you can push them to the online repository where everyone else on the team can access.

For more about GitHub, watch [this video](https://www.youtube.com/watch?v=w3jLJU7DT5E).

#### Getting files on your computer

Once you have created your account and have joined the ISS organization on GitHub, you can actually pull (download) this specific repository from the internet.

In order to do that, you have to set up a personal token that verifies on your local system that you are the correct person trying to access the repository. Check out [this link](https://docs.github.com/en/github/authenticating-to-github/keeping-your-account-and-data-secure/creating-a-personal-access-token) to set up a token for your account. 

When complete run the follwoing commands:

```
$ git clone https://github.com/ISSUIUC/ISS_Learning_Controls.git
Username: your_username
Password: your_token
```

#### Updating your files

Sometimes there are revisions to the program that increase performance or introduce new features. Therefore, everytime you open to use this program, it is best practice pull (download) the most recent software by running the following command:

```
$ git pull origin main
```

## Installing Python

#### Installing Conda

In order to run Python code on your system, you need to install Python itself onto your computer. For this specfic use case, we will use Anaconda due to it's application in future aerospace courses and useful tools included. We recommend using Miniconda for a light installation process.

- [Install on macOS](https://docs.conda.io/projects/conda/en/latest/user-guide/install/macos.html)
- [Install on Linux](https://docs.conda.io/projects/conda/en/latest/user-guide/install/windows.html)
- [Install on Windows](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)

#### Create Conda Enviroment 

Now that you have Python installed on your system, it's time to set up your environments! We use Python environments to contain libraries installed for different projects. Run the follwoing command to create your own Python environment:

```
conda create --name learning_controls python=3
```

Activate your newly created environment:

```
conda activate learning_controls
```

Finally, install the library dependencies using predefined dependencies:

```
pip install -r requirements.txt
```

Just in case, run the following command to make sure Jupyter Notebook is installed:
```
conda install jupyter notebook
````

## Contributors

[Jeffery Zhou](https://github.com/Jeffery-Zhou-98), [Karnap Patel](https://github.com/karnapp2), [Ayberk Yaraneri](https://github.com/AyberkY), [Kenneth Tochihara](https://github.com/ktt3), [Parth Shrotri](https://github.com/parthshrotri)

## References

All of our tutorials for 2022-2023 are based on the inclass examples given by Professor Tim Bretl in AE353 Spring 2022. We are grateful for his support in using these examples to introduce ISS members to GNC. His original code, notes and reference materials can be found [here](https://github.com/tbretl/ae353-sp22).
