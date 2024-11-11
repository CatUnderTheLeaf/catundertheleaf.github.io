---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
---

<!-- what is with that post-title??? -->
## <ins>Computer Vision:</ins>

<!-- musicScanner-->
{% include project.html 
    project_name = "musicScanner"
    project_link = "https://github.com/CatUnderTheLeaf/musicScanner"
    image_path="/assets/musicScanner/object_detection.png" 
    title="Optical Music Recognition using Deep Learning" 
    app_link = "https://musicscanner.streamlit.app/"
    category="musicScanner" 
%}

<!-- getYourLook-->
{% include project.html 
    project_name = "getYourLook"
    project_link = "https://github.com/CatUnderTheLeaf/getYourLook"
    image_path="/assets/gyl/haircutrec.png" 
    title="Haircut recommendations based on the face shape" 
    app_link = "https://getyourlook.streamlit.app/"
    category="gyl" 
%}
---
## <ins>Robotics:</ins>

<!-- deepRacerSim -->
{% include project.html
    project_name = "deepRacerSim"
    project_link = "https://github.com/CatUnderTheLeaf/deepRacerSim" 
    image_path="/assets/deepRacerSim/teleoperation.png" 
    title="Simulation for an AWS DeepRacer car" 
    category="aws" 
%}

<!-- rosRoboCar -->
{% include project.html 
    project_name = "rosRoboCar"
    project_link = "https://github.com/CatUnderTheLeaf/rosRoboCar"
    image_path="/assets/rosRoboCar/donkey2.png" 
    title="Self-driving car for the 'Autonomous Driving Competition'" 
    category="donkey" 
%}
---
## <ins>Automation:</ins>

<!-- telegram_channel_backup -->
{% include project.html 
    project_name = "telegram_channel_backup"
    project_link = "https://github.com/CatUnderTheLeaf/telegram_channel_backup"
    image_path="/assets/telegram.png" 
    title="GitHub Action workflow and Python scripts that back up your telegram channel to GitHub" 
    category="telegram" 
%}

<!-- M.A.Ge -->
{% include project.html 
    project_name = "M.A.Ge"
    project_link = "https://github.com/CatUnderTheLeaf/menuGenerator"
    image_path="" 
    title="An Android app written in python to generate menu for 1 day/week/month." 
    category="mage" 
%}

