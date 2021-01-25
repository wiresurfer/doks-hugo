---
title: "Build Logs"
description:
type: developer-guide
date: 2019-10-25T13:22:44+05:30
pre: "a. "
weight: 515
---
Build logs are generated when rapyuta.io converts the
source code of a git repository into a runnable docker image.

To view build logs of a build on rapyuta.io,
follow these steps:

1. On the left navigation bar, click **BUILDS**.  
2. Select a build from the build list page. Click on **SHOW MORE** and go to **Build history** tab.  
   The tab displays the time of build creation, the author who created it, the build status, and the error string.  
![build Logs](/images/core-concepts/logging/build-logs/build-history-view-logs.png?classes=border,shadow&width=60pc)     
3. To view details of the git repository such as repository’s name, commit number, commit message and the git hosting platform, 
   click **View details/logs**.
4. Click on **View logs**, to see the build logs.
5. Build logs are displayed in the logging area (terminal-like window). You may
   scroll up to view the starting logs of the log stream.

Once all of the build logs are generated, the status indicator greys out and
you'll see a **Logs streaming ended** status (top left corner of the terminal
window). If you see an error and the logs are not being generated,
click **Refresh**.

![build Logs](/images/core-concepts/logging/build-logs/build-logs.png?classes=border,shadow&width=60pc)

