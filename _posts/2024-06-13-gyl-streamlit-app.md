---
layout: post
title: "getYourLook - Haircut recommendations"
subtitle: "getYourLook WebApp"
categories: gyl
---
## Roadmap
{% assign posts = site.categories["gyl"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#mvp">MVP</a></li>
               <li><a href="#architecture">Architecture</a></li>
               <li><a href="#future-improvements">Future improvements</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## MVP

The idea was to make a Minimal Viable Product - a simple WebApp where user can take/upload photo and recieve haircut recommendations. To make such a product I had to search for a solution that meets my requirements:
- fast to make a working app
- zero cost
- easy to deploy and maintain 

[Streamlit](https://streamlit.io/) met all my requirements:
- it is so easy to write UI elements as python code, no need to think about implementing it
- you can deploy Streamlit App wherever you want, but you have an option to deploy to the Streamlit Community Cloud for free, where you can use:
  - CPU: 0.078 cores minimum, 2 cores maximum
  - Memory: 690MB minimum, 2.7GBs maximum
  - Storage: No minimum, 50GB maximum
- it is very easy to deploy - just connect your GitHub account and point to the repository and branch, each push commit will evoke a webhook to update code

Current WebApp is available at [getyourlook.streamlit.app](https://getyourlook.streamlit.app/)

## Architecture

![app architecture](/assets/gyl/architecture.png)
*diagram was made with [excalidraw.com](https://excalidraw.com/)*

All models and files are cached to save resources and for the app to have quick response.

## Future improvements

__Apply style to the photo__

As it is a MVP, the recommeded haircuts are in the form of images manually generated with [stability-ai](https://replicate.com/stability-ai/sdxl). To make WebApp more user-oriented images should be generated from the input image and recommendation prompts, so the user can see not some random images, but all recommended haircuts applied to his/her image.

__Men haircuts__

Right now only female haircuts are shown and the model was trained only on female celebrities. The model should be retrained also with male faces, and haircut recommendations should include more variety of haircuts.

__Hair type__

It would be cool to take into account hair type, because if the user has curly hair, why recommend a haircut with straight hair, also some haircuts look differently depending on the thickness of hair.

__Sun-glasses shape__

Face shape can help not only with a haircut, but also with the shape of sun-glasses or how to apply a make-up.

__Color palette__

Ideally the face also is analysed for skin/eyes/hair color. The corresponding color palette is recommended based on 'spring', 'summer', 'autumn' or 'winter' types. For this task additional model should be trained or current model should be changed to the MTCNN.

__Camera calibration and distortion__

It seems that shape classification also depends on camera distortion. As all users have different cameras it is impossible to apply some random hardcoded undistortion. Maybe camera information can be retrived from the image and matrices for undistortion can be generated.


<!-- https://priverevaux.com/blogs/eyewear/glasses-for-face-shape
https://github.com/Laksh1701/Spectacles-recommendation-system-based-on-faceshape
https://www.occhialando.de/blog-optische-brillen-mode/ratgeber-zur-wahl-der-perfekten-brille
https://www.pinterest.com/pin/155585362115815230/ -->