---
layout: post
title: "telegram_channel_backup. How it works"
subtitle: "How it works"
categories: telegram
---

### Action workflow

- action is triggered on schedule at 23:30 UTC every day
- installs `python-telegram-bot` and `PyGithub`
- makes `_posts` folder for posts
- run `backup_telegram_channel.py` with usage of `BOT_TOKEN`
- run `upload_to_github.py` with usage of `AUTH_TOKEN`

### backup_telegram_channel.py

- gets channel updates for last 24 hours and saves as dictionary of the newly created or the last version of updated posts, uses python-telegram-bot library
  > `python-telegram-bot` is a wrapper of the [Telegram Bot API](https://core.telegram.org/bots/api). Incoming channel updates are stored on the server **no longer** than 24 hours. That's why action in the orkflo is triggered once every day. If you need more functionality, then look into full [Telegram API](https://core.telegram.org/api#telegram-api), register new Telegram App and use `api_id` and an `api_hash` for authentification. This script should be rewritten for usage of another API.
- save all posts from dictionary in the `YYYY-MM-DD-postId.md` format in `_posts` folder

### upload_to_github.py

- gets all files in your channel backup repository as a set
- commits and pushes all posts from `_posts` folder with right commit message (updated or created post)

### (Optional, not used in the action workflow) backup_telegram_channel_history.py

If you already have a channel and there are already posts older than last 24 hours, then this optional script is for you. You need to have Python3 and Telegram Desktop installed. Telegraph and BeautifulSoup are used to prettify html string from Telegraph articles.

- Checkout this repository and install necessary dependancies:
   {% highlight shell %}
   $ git clone https://github.com/CatUnderTheLeaf/telegram_channel_backup.git
   
   $ pip install PyGithub
   $ pip install 'telegraph[aio]'
   $ pip install beautifulsoup4
   
   # create folder for old posts in json format
   $ cd telegram_channel_backup
   $ mkdir json_dump
   {% endhighlight %}
- Go to Telegram Desktop, open your channel, click on three dots sign in the upper right corner and select `export chat history`, select `JSON` as format and `json_dump` as path
- Convert posts and upload them to your repository:
   {% highlight shell %}
   # convert json format to posts in the `YYYY-MM-DD-postId.md` format in `_posts` folder
   # json_dump is a folder with dumped history and images
   $ python backup_telegram_channel_history.py dump_dir=json_dump
   
   # replace placeholders for actual values
   $ python upload_to_github.py auth_token=$AUTH_TOKEN repo=$CHANNEL_REPO branch=$BRANCH
   {% endhighlight %}