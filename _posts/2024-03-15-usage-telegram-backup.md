---
layout: post
title: "telegram_channel_backup. How to use"
subtitle: "How to use"
categories: telegram
---

The idea is very simple: you have a Telegram channel and just want to automatically backup your posts to GitHub.

Sounds easy. But how to do this with less hassle and without registering of additional Telegram and GitHub Apps IDs? Just follow these steps.

> Now it only backups text posts, post-images, text posts with images and also downloads your telegraph articles in `html` format. Update for backuping posts with other types of media will come later.

## How to use

### 1. Create a Telegram bot and get its token

- In Telegram find `@BotFather`, Telegram’s tool for creating and managing bots.
- Use the `/newbot` command to create a new bot.
- `@BotFather` will ask you for a name and username, then generate an authentication token for your new bot.

   > The 'name' of your bot is displayed in contact details and elsewhere.

   > The 'username' is a short name, used in search, mentions and t.me links. Usernames are 5-32 characters long and not case sensitive – but may only include Latin characters, numbers, and underscores. Your bot's username must end in 'bot', like 'tetris_bot' or 'TetrisBot'.

   > The 'token' is a string, like `110201543:AAHdqTcvCH1vGWJxfSeofSAs0K5PALDsaw`, which is required to authorize the bot and send requests to the Bot API. Keep your token secure and store it safely, it can be used by anyone to control your bot.

   > Unlike the bot’s name, the username cannot be changed later – so choose it carefully.

- Add newly created bot to your channel as subscriber or administrator.

### 2. Create a (private) repository for channel backup

### 3. Create a personal access token (lets call it `AUTH`)

Just follow the instructions. Be sure to grant repository read-and-write permissions for `actions` and `contents`. You can also restrict permissions to only this repository.

1. Verify your email address, if it hasn't been verified yet.
2. In the upper-right corner of any page, click your profile photo, then click Settings.
3. In the left sidebar, click  Developer settings.
4. In the left sidebar, under  Personal access tokens, click Fine-grained tokens.
5. Click Generate new token.
6. Under Token name, enter a name for the token.
7. Under Expiration, select an expiration for the token.
8. Optionally, under Description, add a note to describe the purpose of the token.
9. Under Resource owner, select a resource owner. The token will only be able to access resources owned by the selected resource owner. Organizations that you are a member of will not appear unless the organization opted in to fine-grained personal access tokens. For more information, see "Setting a personal access token policy for your organization."
10. Optionally, if the resource owner is an organization that requires approval for fine-grained personal access tokens, below the resource owner, in the box, enter a justification for the request.
11. Under Repository access, select which repositories you want the token to access. You should choose the minimal repository access that meets your needs. Tokens always include read-only access to all public repositories on GitHub.
12. If you selected Only select repositories in the previous step, under the Selected repositories dropdown, select the repositories that you want the token to access.
13. Under Permissions, select which permissions to grant the token. Depending on which resource owner and which repository access you specified, there are repository, organization, and account permissions. You should choose the minimal permissions necessary for your needs.
14. Click Generate token.

### 4. Add `BOT` and `AUTH` tokens to your channel backup repository as secrets

Follow these instructions and add `AUTH_TOKEN` and `BOT_TOKEN` secrets:

1. On GitHub.com, navigate to the main page of the repository.
2. Under your repository name, click  Settings. If you cannot see the "Settings" tab, select the  dropdown menu, then click Settings.
3. In the "Security" section of the sidebar, select  Secrets and variables, then click Actions.
4. Click the Secrets tab.
5. Click New repository secret.
6. In the Name field, type a name for your secret.
7. In the Secret field, enter the value for your secret.
8. Click Add secret.

### 5. Create an action workflow

- Copy code from [action_workflow_example](action_workflow_example.yml)

{% highlight yml %}
# action_workflow_example.yml
name: backup workflow

on:
  schedule:
    - cron: '30 23 * * *'
  
permissions: write-all

# replace with your names
env:
  CHANNEL_REPO: your repository name
  BRANCH: branch name

jobs:
  import:
    runs-on: ubuntu-latest
    steps:
      - name: Install Dependencies
        run: |
          pip install python-telegram-bot --upgrade
          pip install PyGithub
          pip install 'telegraph[aio]'
          pip install beautifulsoup4
          
      - name: Checkout telegram_channel_backup repo
        uses: actions/checkout@v4
        with:
          repository: CatUnderTheLeaf/telegram_channel_backup
          path: telegram_channel_backup

      - name: Run backup
        env:
          BOT_TOKEN: ${{ secrets.BOT_TOKEN }}
        run: python telegram_channel_backup/backup_telegram_channel.py bot_token="$BOT_TOKEN"

      - name: Run upload
        env:
          AUTH_TOKEN: ${{ secrets.AUTH_TOKEN }}
        run: python telegram_channel_backup/upload_to_github.py auth_token="$AUTH_TOKEN" repo=$CHANNEL_REPO branch=$BRANCH
{% endhighlight %}
- Replace `CHANNEL_REPO` and `BRANCH` placeholders with your names
- Action is by default triggered on schedule at 23:30 UTC every day.

If you want to change it, here is short informatio about cron syntax.

Cron syntax has five fields separated by a space, and each field represents a unit of time.
{% highlight shell %}
┌───────────── minute (0 - 59)
│ ┌───────────── hour (0 - 23)
│ │ ┌───────────── day of the month (1 - 31)
│ │ │ ┌───────────── month (1 - 12 or JAN-DEC)
│ │ │ │ ┌───────────── day of the week (0 - 6 or SUN-SAT)
│ │ │ │ │
│ │ │ │ │
│ │ │ │ │
* * * * *
{% endhighlight %}

You can use these operators in any of the five fields:

| Operator | Description | Example |
| ---| --- | --- |
| *	| Any value	| 15 * * * * runs at every minute 15 of every hour of every day |
| ,	| Value list separator	| 2,10 4,5 * * * runs at minute 2 and 10 of the 4th and 5th hour of every day |
| -	| Range of values | 30 4-6 * * * runs at minute 30 of the 4th, 5th, and 6th hour |
| /	| Step values | 20/15 * * * * runs every 15 minutes starting from minute 20 through 59 (minutes 20, 35, and 50) |

You can use [crontab guru](https://crontab.guru/) to help generate your cron syntax and confirm what time it will run. To help you get started, there is also a list of [crontab guru examples](https://crontab.guru/examples.html).

{% assign posts = site.categories["telegram"] | sort %}
{% for post in posts %}
   {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}         
   {% endif %}
{% endfor %}

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>