WHAT IS THIS?
=============

Linux Kernel source code for the bq Newton device. 


BUILD INSTRUCTIONS?
===================

Specific sources are separated by branches and each version is tagged with it's corresponding number. First, you should
clone the project:

	$ git clone git@github.com:bq/newton.git

After it, choose the version you would like to build:

	$ cd newton/
	$ git checkout Oma.JBv1.2-26.09.2012


Finally, build the kernel according to the defconfig file: newton_defconfig 

	$ cd kernel/
	$ make newton_defconfig
	$ make kernel.img
