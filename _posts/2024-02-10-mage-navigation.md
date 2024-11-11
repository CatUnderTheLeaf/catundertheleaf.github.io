---
layout: post
title: "M.A.Ge - your automated menu generator"
subtitle: "App navigation"
categories: mage
---

## Roadmap
{% assign posts = site.categories["mage"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#menu">Menu</a></li>
               <li><a href="#navigation">Navigation</a></li>
               <li><a href="#settings">Settings</a></li>
               <li><a href="#recipes">Recipes</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

> The app is available for free on [Amazon Appstore](https://www.amazon.de/gp/product/B09YYXN35D)

## Menu

On first launch app opens a page with a menu generated for a week. In the upper area there are two buttons: `Settings` and `Regenerate`. Each date is a separate tab, tabs can be scrolled. Also users can swipe left or right within the content area to navigate between tabs.

In the content area we see:
- name of the meal type (breakfast, lunch, dinner, brunch, supper)
- image of the meal
- clock that indicates time to prepare this meal
- name of the meal with a short ingridients list
- click on meal's name drop downs a full ingridients list and prepare instructions

<p align="center">
  <img src="/assets/mage/Menu.png" width="300" title="Menu">
  <img src="/assets/mage/Menu_recipe.png" width="300" title="Menu_recipe">
</p>

## Navigation

Access to other pages is made through Navigation drawer. It has only three pages: `Menu`, `Settings` and `Recipes`. 

<p align="center">
  <img src="/assets/mage/Navigation_drawer.png" width="300" title="Navigation drawer">
</p>

## Settings

User can set:
- __Time period__ - generate a menu for a day/week/month
- __Repeat dishes__ - indicates if food is cooked in large batches and leftovers can be eaten on the next day
- __Meals__ - how many meals per day user wants in the menu

<p align="center">
  <img src="/assets/mage/Settings.png" width="300" title="Settings">
</p>

There are four rules user can set:
- set __nutrients__ for each meal type, e.g. only carbs for breakfast and protein for dinner, veggies for all meal types.
- set __preparation time__ for each day, e.g. on Mo-Fr get meals with short preparation time, as you have work.
- __discard__ some meals, e.g. every Su you have a lunch at the restaurant, so no meal is needed
- set __tags__ for each meal type, e.g. for breakfast get only recipes with 'breakfast' tag, so there will be no soup so early

<p align="center">
  <img src="/assets/mage/Settings_rules1.png" width="300" title="Settings_rules1">
  <img src="/assets/mage/Settings_rules2.png" width="300" title="Settings_rules2">
  <img src="/assets/mage/Settings_rules3.png" width="300" title="Settings_rules3">
  <img src="/assets/mage/Settings_rules4.png" width="300" title="Settings_rules4">
</p>

`Help` button in the right upper corner shows what pictograms mean.

## Recipes

In `Recipes` you can:
- __view__ a list of recipes - 64 basic recipes of Eastern European cuisine are already included
- __create__ with a `+` button
- __delete__ recipes with long touch on recipes name
- __edit__ recipes with a short touch on recipes name

<p align="center">
  <img src="/assets/mage/All_recipes.png" width="300" title="All_recipes">
  <img src="/assets/mage/Delete_recipe.png" width="300" title="Delete_recipe">
</p>

On the `Add/Edit Recipe` user can:
- set __title__
- select __preparation time__
- set __image__ from gallery or camera
- set __ingredients__
- mark recipe for '__repeat__', so the recipe can be used on two consecutive days
- add __tags__

<p align="center">
  <img src="/assets/mage/Edit_recipe.png" width="300" title="Edit_recipe">
  <img src="/assets/mage/Choose_image.png" width="300" title="Choose_image">
  <img src="/assets/mage/Add_ingredients.png" width="300" title="Add_ingredients">
  <img src="/assets/mage/add_tags.png" width="300" title="add_tags">
</p>