---
title: "Failure Codes"
description:
type: developer-guide
date: 2019-10-24T11:49:07+05:30
pre: "1. "
weight: 255
---
A device may ***fail*** due to several reasons. The below table
lists all the possible error codes together with their
description and recommended actions.

If the issue still persists, please <a href="#" onclick="javascript:FreshWidget.show();">contact support</a>.

<table>
    <thead>
        <tr>
            <th>Error Code</th>
            <th>Description</th>
            <th>Recommended Actions</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>DEV_E100</td>
            <td>Internal error</td>
            <td><a href="#" onclick="javascript:FreshWidget.show();" class="highlight">Contact support</a></td>
        </tr>
        <tr>
            <td>DEV_E101</td>
            <td>Downloading internal artifact failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E102</td>
            <td>Pulling internal docker image failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                    <li>Ensure that the docker version on device is:
                        <ul>
                            <li><strong><em>docker-ce==17.12.1~ce-0~ubuntu</em></strong> for Ubuntu 16.04</li>
                            <li><strong><em>docker-ce==5:18.09.7~3-0~ubuntu-bionic</em></strong> for Ubuntu 18.04</li>
                        </ul>If not, uninstall it and <a href="/developer-guide/manage-machines/onboarding/setup-device/" class="highlight">set up the device</a> again.</li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E103</td>
            <td>Installing pip package failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                    <li>Ensure that the version of Python on device is <strong><em>Python&gt;=2.7.8,&lt;3</em></strong><br>
                    If not, uninstall it and <a href="/developer-guide/manage-machines/onboarding/setup-device/" class="highlight">set up the device</a> again.</li>
                    <li>Try installing the package manually by executing<br>
                    <code>source /opt/rapyuta/venv/bin/activate &amp;&amp; <br>pip install &lt;python-package&gt;</code> and see if it fails.<br>
                    If it doesn’t fail, the issue may be transient.<br>
                    Now, <a href="/developer-guide/manage-machines/onboarding/setup-device/">set up the device</a> on successful installation of python package.</li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E104</td>
            <td>Installing docker failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                    <li>Ensure that the docker version on device is:
                        <ul>
                            <li><strong><em>docker-ce==17.12.1~ce-0~ubuntu</em></strong> for Ubuntu 16.04</li>
                            <li><strong><em>docker-ce==5:18.09.7~3-0~ubuntu-bionic</em></strong> for Ubuntu 18.04</li>
                        </ul>If not, uninstall it and <a href="/developer-guide/manage-machines/onboarding/setup-device/" class="highlight">set up the device</a> again.</li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E105</td>
            <td>Installing system package failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                    <li>Try installing the package manually by executing<br>
                    <code>apt-get install &lt;system-package&gt;</code> and see if it fails. If it doesn’t fail, the issue may be transient.<br>
                    Now, <a href="/developer-guide/manage-machines/onboarding/setup-device/">set up the device</a> on successful installation of system package.</li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E106</td>
            <td>Managing files on device failed</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li><a href="#check-for-disk-space" class="highlight">Ensure that there is adequate disk space on device.</a></li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E107</td>
            <td>Internal service failed to start</td>
            <td><a href="#" onclick="javascript:FreshWidget.show();" class="highlight">Contact support</a></td>
        </tr>
        <tr>
            <td>DEV_E108</td>
            <td>Initialization time exceeded</td>
            <td>
                <ul>
                    <li><a href="/developer-guide/manage-machines/onboarding/setup-device/" class="highlight">Set up the device</a> again.</li>
                    <li><a href="#" onclick="javascript:FreshWidget.show();" class="highlight">Contact support</a></li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E109</td>
            <td>Initialization failed due to network error</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                </ul>
            </td>
        </tr>
        <tr>
            <td>DEV_E110</td>
            <td>Initialization failed due to docker login error</td>
            <td>
                <ul>
                    <li><a href="#check-for-active-internet-connection" class="highlight">Ensure that device has an active internet connection.</a></li>
                    <li>Ensure that the package <strong><em>golang-docker-credential-helpers</em></strong> is not installed. If present remove it manually <br>
                     by executing <code>apt-get remove golang-docker-credential-helpers</code>. Now, <a href="/developer-guide/manage-machines/onboarding/setup-device/" class="highlight">set up the device</a> again.</li>
                </ul>
            </td>
        </tr>
    </tbody>
</table>

#### Check for active internet connection
To check if a device has an active internet connection, execute the command:
`ping -c 4 8.8.8.8` at the device's terminal.
The *[ping](https://linux.die.net/man/8/ping)* command will try
to reach [Google Public DNS](https://en.wikipedia.org/wiki/Google_Public_DNS).
If successful, you should see an output as:
***4 packets transmitted, 4 received, 0% packet loss.***

#### Check for disk space
To check if there is adequate disk space on device, run the command: `df -h`
at the device's terminal. The *[df](https://linux.die.net/man/1/df)* command
will display your disk usage. Ensure the value of percentage used for
your main filesystem is not nearly 100%.