Setup local git repositories
============================

In HOME, create the seabot2 folder which will contain all the seabot2 related softwares.

.. code-block:: bash

    cd $HOME
    mkdir seabot2
    cd seabot2
    

Clone necessary git repositeries among the following :

.. code-block:: bash

    git clone https://github.com/emoi-idgf/seabot2-tools.git
    git clone https://github.com/emoi-idgf/seabot2-ros.git
    git clone https://github.com/emoi-idgf/seabot2-docker.git
    git clone https://github.com/emoi-idgf/seabot2-docs.git
    git clone https://github.com/emoi-idgf/seabot2-linux.git
    git clone https://github.com/emoi-idgf/seabot2-qgis.git
    git clone https://github.com/emoi-idgf/seabot2-hardware.git
    

Init seabot2-ros submodules :

.. code-block:: bash
    
    cd seabot2-ros
    git checkout devel
    git submodule update --init --recursive

