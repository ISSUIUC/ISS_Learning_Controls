# ISS_Learning_Controls
Learn the fundamentals of controls!

## Purpose

The purpose of this repository is to educate Illinois Space Society (ISS) members about controls. This repository includes resources ande demos in which members can learn and apply control theory. The resources provided allow for learning the basics of controls theory. The demo is a playground to tinker with the control system, and get immediate feedback on the quality of system created. This repository contains all the demos and resources necessary to get a basic understanding of controls to be applied to numerous projects in ISS.

## File Structure

#### Basics of Terminal Commands

Programming is extremly heavy with the command line usage, whether that be macOS, Linux, or Windows. Before moving on, review these quick tutorials based on your system. 

:warning: **If you are using Windows, please make sure to use PowerShell as your terminal!**

- [Tutorial](https://github.com/ISSUIUC/ISS_Learning_Controls.git)
- [Cheat Sheet](https://www.guru99.com/linux-commands-cheat-sheet.html)

#### What are in these files?

`controls-demo` contains the control code and simulations. This is where your sandbox for your control code will live. 

`resources` contains numerous resources from notes taken from YouTube tutorials. Also contains links to useful tutorials we've used to learn control theory.

`requirements.txt` contains all the necessary packages required to run the control simulation. Use of this file will be further explained later. 

## Using GitHub

#### What is GitHub?

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

## References

This repository was created using Professor Bretl's [AE 353 Spring 2021 Controls code base](https://github.com/tbretl/ae353-sp21).
