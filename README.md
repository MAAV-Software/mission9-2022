# MAAV Mission 9 Repo - 2021

## Introduction
This README document is a work-in-progress. We will populate this more later.

**Note: this branch is only up to date for WSL users, use with caution with other platforms.**

## Install and Simulator Setup Instructions

Let MAAV Software Leadership know if you run into issues during any of the setup.

[Windows 10 / WSL 2](docs/WSL.md)

[Mac](docs/Mac.md)

[Ubuntu / Other Linux Distro](docs/Linux.md)

[Docker Additional Info](docs/Docker.md)

## Development Etiquette

This section will go over our team's expectations with using Git/Github.
We will also provide a summary of common Git operations in case you are
new to using Git with software projects. If you need help with Git/Github,
reach out to the team in Slack.

When in doubt, [check out "Oh Shit! Git!"](https://ohshitgit.com)

### Git Identity

Make sure your identity through Git is setup well.

```bash
# List current global config, which may contain identity info

$ git config --global -l
user.name=Jacob Minock
user.email=jaminock@umich.edu
core.editor=vim
...

# Set or Change some things (for any repository you use)
# Omit --global to just make it for THIS repository

$ git config --global user.name "Jacob Minock"
$ git config --global user.email "jaminock@umich.edu"
```

### Git Branches

When developing a new feature, bug fix, or set of changes, make a new branch.
This will allow your `master` branch to stay clean and organized in terms of
commit history and file changes. If you are added to the MAAV-Software organization,
it will also allow you to push changes to this project and make pull requests
without needing to fork the repository.

This project's `master` branch is protected, and only Software Leadership can merge
in changes from other branches or forks (more on forks later).

```bash
# List branches
$ git branch

# Create a new branch
$ git checkout -b <branch>

# Switch to a branch
$ git checkout <branch>

# Delete a branch (will warn you if branch is not fully merged into master)
$ git branch -d <branch>
```

### Git Repository Fork

Simple GitHub Forking Tutorial:

https://guides.github.com/activities/forking/

Forking repositories is a straightforward approach to making changes to shared repositories.
'Forked' repositories are personal copies of a shared project, in which you can make changes
and then offer those changes up to the original project.

If you are officially a member of MAAV Software, forking the repo is optional. However, forking
gives you access control over your pushed changes - whereas anyone else on MAAV Software
can view and make changes to *any* branch that is not `master`.

## Simulating the Vehicle

We will be using Docker to simulate the environment that will be on the
vehicle's computing hardware. To setup the Docker image (a "snapshot" of the
computer at a specific time and state) for this repo, follow
[these instructions](docs/Docker.md) or the instructions specific to your system.

We will also be using Gazebo to simulate vehicle behavior. This is done within
the Docker container to ensure dependencies are setup correctly in our code.
Gazebo is already installed for you if you follow the install instructions for
your system.
