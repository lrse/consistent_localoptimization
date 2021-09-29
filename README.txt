# Consistent SLAM using Local Optimization with Virtual Prior Topologies
This is the official code release for:

#### Consistent SLAM using Local Optimization with Virtual Prior Topologies (IROS 2021)

[Gastón Castro](gcastro@cifasis-conicet.gov.ar), [Facundo Pessacg](fpessacg@dc.uba.ar), [Pablo De Cristóforis](pdecris@dc.uba.ar)

## Notice

The proposed method is implemented using the SLAM Toolbox for MATLAB, originally available at [https://github.com/joansola/slamtb](https://github.com/joansola/slamtb).
Credits and rights of the SLAM Toolbox for Matlab are for its creators and maintainers.
We greatly value their contribution for making it available.

The proposed method implementation is written over the 'graph' branch of the slamtb project.
Folder 'LocalOptimization' holds the majority of introduced changes.

## Usage

Code was tested in Ubuntu 18 with MATLAB R2018a.
In the Matlab prompt:  

  1. Go to the toolbox 
        
        >> cd slamtb

  2. Add all subdirectories in slamtb/ to your Matlab path using the provided script: 
        
        >> slamrc

  3. Run script applying the local optimization with virtual priors approach

        >> slamtb_localgraph
        
  4. Run script applying the global optimization approach

        >> slamtb_graph
        
The simulated scenario parameters can be edited in userDataGraph.m.
Scenarios used in the paper can be selected with the scene parameter:

    scene = 'simpleReverseSquare'; % 'simpleSquare', 'quadraSquare', 'simpleReverseSquare'
  
The local optimization approach with a marginal obtained using the Schur complement can be activated setting the 'SchurOut' topology:

    'topology',       'chain'));     % type of topology: chain, SchurOut

# Copyright and license of the original SLAM Toolbox for Matlab project

=========================
(c) 2007, 2008, 2009, 2010  Joan Sola  @ LAAS-CNRS; 
(c) 2010, 2011, 2012, 2013  Joan Sola
(c) 2014, 2015, 2016        Joan Sola  @ IRI-UPC-CSIC; 
(c) 2009  Joan Sola, David Marquez, Jean Marie Codol,
          Aurelien Gonzalez and Teresa Vidal-Calleja, @ LAAS-CNRS; 

Maintained by Joan Sola
Please write feedback, suggestions and bugs to:

    jsola@iri.upc.edu

or use the GitHub web tools.

Published under GPL license. See COPYING.txt. 


