
XPS-FORK
===========================================

<br>**If you like my work, don't hesitate to donate- [Paypal-Me](https://paypal.me/ARUNVARADARAJAN)** <br>

* This fork is maintained to support HKG community. The default branch tracks
the latest Master-ci updates.

* This fork has support for MDPS Harness<sup>^^</sup> , OP Long<sup>**</sup>, Non-SCC cars, anti-nag for panda only setup

Notes -

* Please feel free to add PR for support for non existing FP.

* If you encounter "Car Unrecognized- Please verify fingerprint with the fork owner"
  message on the screen, please contact me directly on discord for help.

* Even though the configuration is auto detected, there may be CAN ERROR in some cases,
please contact me directly on discord for help.

* This branch uses a different tuning of PI controller for lat(steering). There is a
non linear term added with respect to steering request which results in a smoother
behavior.

* This branch is also fine tuned to have smoother OP long(gas/brake) control.<sup>**</sup>

* **!DANGER!** OP long with radar disable method will disable stock AEB/FCA completely.

-----------------------------------------------------------
<sup>**</sup> OP long requires one of the below modifications-

* Radar harness to move radar to CAN bus 2 (tested)
* Non-SCC vehicle variant coded to accept accel request (non proven/ non tested)
* Radar_disable UDS command (tested but not recommended), the feature is hardcoded to False for safety

___________________________________________________________
**!!! ENABLING LOGIC FOR MDPS HARNESS MAY GET YOU BANNED !!!**
<br>
<br>
<sup>^^</sup> There are 2 types of MDPS Harnesses that work with this fork-

* MDPS Harness connected to CAN1 at the comma harness replacing LCAN (**type1**)
* MDPS Harness connected to OBD/Comma power(**type2**) - **most popular**

Important step to enable MDPS Harness functionality if you have steering fault light
<br>
[![](https://imgur.com/gVwuVJQ.png)](#)

* Go to developer settings and toggle the MDPS Harness on
* Make sure device is connected to the car
* Select the MDPS harness type, the MDPS Harness should automatically switch and flash Panda.
* Wait for the reboot to complete.

[![](https://imgur.com/K4QumwC.png)](#)
----------------------------------------------------------  

* If you have git pull or update issues -
  - Run a clean and reset command - `cd /data/openpilot && git clean -xdf && git reset --hard`
  - Retry git pull - `cd /data/openpilot && git pull`

___________________________________________________________

* Check out OP-EDIT to edit tuning set up -
  - To open OP-EDIT - `cd /data/openpilot && python op_edit.py`
  - Enter your user name to track your changes.
  - Modify values as required.

**If you like my work, don't hesitate to donate- [Paypal-Me](https://paypal.me/ARUNVARADARAJAN)**
