# Snowplow2025
2024-2025 Hy-flex Code for ION Autonomous Snowplow

## Getting Started

### Setup your ssh keys
Follow these instructions: https://help.github.com/articles/connecting-to-github-with-ssh/

### Clone the repo
There are certain scripts that assume that you have the Snowplow2025 repo installed in your home directory. It is recommended you don't try to clone the repo anywhere else.
```
cd ~
git clone git@github.com:BisonRobotics/Snowplow2025.git
```
## Development Conventions

### Style Guidelines

#### ROS
See: http://wiki.ros.org/ROS/Patterns/Conventions
And: http://wiki.ros.org/StyleGuide

#### C++
See: http://wiki.ros.org/CppStyleGuide
And: https://google.github.io/styleguide/cppguide.html

#### Python
See: http://wiki.ros.org/PyStyleGuide

### Unit Testing
See: http://wiki.ros.org/Quality/Tutorials/UnitTesting

## Misc Info

### Motor Controllers
[Roboteq HDC2460](https://www.roboteq.com/products/products-brushed-dc-motor-controllers/hdc2450-259-detail) \
[Roboteq User Manual](https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file)

### Onboard computer login information

name: \
server: \
user: \
pass: \
address:

Common hosts [ref](https://docs.google.com/document/d/1SIL_rD9zDHXBfXHYZ7J_V_AXUjoopJnW3r_VlywnoGs/edit?usp=sharing) :
access point: `192.168.1.10`
Computer: `192.168.1.1`

## Deployment

### Where to Deploy

Most changes will need to be deployed to the main computer. Changes to the jetson_pkg package will require a deployment to the Jetson and not to the main computer.

### Main Computer

Connect to the main computer using WinSCP. Drag and drop this entire project into the main computer's file system. It will likely ask you if you want to overwrite the existing directory with the same name. If so, select yes.

### Jetson

Connect to the Jetson using WinSCP. Drag and drop this entire project into the Jetson's file system. It will likely ask you if you want to overwrite the existing directory witht he same name. If so, select yes.
