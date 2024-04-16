---
layout: page
title: Projects
permalink: /projects/
---

### [telegram_channel_backup](https://github.com/CatUnderTheLeaf/telegram_channel_backup)
Backup your telegram channel using GitHub actions

### [deepRacerSim](https://github.com/CatUnderTheLeaf/deepRacerSim)
Simulation for an AWS DeepRacer car
{% assign posts = site.categories["aws"] | sort %}
<ul>
    {% for post in posts %}      
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
    {% endfor %}
</ul>

### [rosRoboCar](https://github.com/CatUnderTheLeaf/rosRoboCar)
Self-driving car for the "Autonomous Driving Competition"

### [M.A.Ge](https://github.com/CatUnderTheLeaf/menuGenerator)
Your automated menu generator
