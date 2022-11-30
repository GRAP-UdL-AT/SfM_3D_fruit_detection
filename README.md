# Fruit detection and 3D location in photogrammetry-derived 3D point clouds

## Introduction
This project is a matlab implementation to project image detections (instance segmentation masks) onto 3D point clouds generated using structure-from-motion. This code was used in [[1]]( https://doi.org/10.1016/j.compag.2019.105165) to locate fruits by using the [Fuji-SfM dataset]( https://doi.org/10.1016/j.dib.2020.105591). Find more information in:
* [Fruit detection and 3D location using instance segmentation neural networks and structure-from-motion photogrammetry [1]](https://doi.org/10.1016/j.compag.2019.105165).

## Preparation 

First of all, create a new project folder:
```
mkdir new_project
```

Then, clone the code inside “new_project” folder:
```
cd new_project
git clone https://github.com/GRAP-UdL-AT/SfM_3D_fruit_detection.git
```

### Prerequisites

* MATLAB R2021a (we have not tested it in other matlab versions)
* Computer Vision System Toolbox
* Statistics and Machine Learning Toolbox

### Data Preparation

Inside the “new_project” folder, save the dataset folder “data” available at [download data]( http://gofile.me/73Gps/LwEKIW4gd). 

### Launch the code

* Execute the file `/new_project/ SfM_3D_fruit_detection/ main_SfM_3D_fruit_detection.m `


## Authorship

This project is contributed by [GRAP-UdL-AT](https://www.grap.udl.cat/en/).

Please contact authors to report bugs @ jordi.genemola@udl.cat

### Citation

If you find this implementation or the analysis conducted in our report helpful, please consider citing:

    @article{Gené-Mola2021a,
        Author = {{Gen{\'e}-Mola, Jordi and Sanz-Cortiella, Ricardo and Rosell-Polo, Joan R and Morros, Josep-Ramon and Ruiz-Hidalgo, Javier and Vilaplana, Ver{\’o}nica and Gregorio, Eduard },
        Title = { Fruit detection and 3D location using instance segmentation neural networks and structure-from-motion photogrammetry },
        Journal = {Computers and Electronics in Agriculture},
        Year = {2020}
        doi = {https://doi.org/10.1016/j.compag.2019.105165 }
    }
    
    @article{Gené-Mola2021b,
        Author = {{Gen{\'e}-Mola, Jordi and Sanz-Cortiella, Ricardo and Rosell-Polo, Joan R and Morros, Josep-Ramon and Ruiz-Hidalgo, Javier and Vilaplana, Ver{\’o}nica and Gregorio, Eduard },
        Title = { Fuji-SfM dataset: A collection of annotated images and point clouds for Fuji apple detection and location using structure-from-motion photogrammetry},
        Journal = {Data in Brief},
        Year = {2020}
        doi = { https://doi.org/10.1016/j.dib.2020.105591}
    }


#### Acknowledgements
This work was partly funded by the Spanish Ministry of Science, Innovation and Universities (grant RTI2018-094222-B-I00[[PAgFRUIT project]]( https://www.pagfruit.udl.cat/en/) by MCIN/AEI/10.13039/501100011033 and by “ERDF, a way of making Europe”, by the European Union).
