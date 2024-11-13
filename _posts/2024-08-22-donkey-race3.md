---
layout: post
title: "rosRoboCar: How it works"
subtitle: "How it works"
categories: donkey
---
## Roadmap
{% assign posts = site.categories["donkey"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#ros-graph">ROS Graph</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## ROS Graph

![app architecture](/assets/rosRoboCar/architecture.png)
*diagram was made with [excalidraw.com](https://excalidraw.com/)*

To detect a middle line of the road I used a FCNN trained from scratch.