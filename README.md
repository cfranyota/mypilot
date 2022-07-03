  ![C2picture](https://user-images.githubusercontent.com/46506059/176013252-b8b7a074-fb64-4403-badb-e9fecabbda4f.jpg)
*Based off of Keegans stuff*
*This 0.8.14 fork includes the following additions*:

- *NEW* Hands on the Wheel monitoring
- *NEW* Speed Limit Control (with offset) @move-fast
- *NEW* Map Data Turn Control (either SIM card data or tethered data from your phone)
- 3 follow distances, 1.8, 1.45, 1.2 m/s (Toyota only) (Thanks @Keegan)
- Red mph when braking (Thanks @alexandresato) also for Honda's :-)
- Legacy engage/disgage sounds
- NEOS 20 for bootloop fix (Thanks @Erich)
- *Torque controller* for select cars, (mostly Toyotas and Hondas)
- 💯 degree steering fault fix (Thanks @sshane)
- Device turns off 3 hours after shutting the car down
- Modded fw detection and tuning for eps mod, Civics / CRV.

- If this fork has helped you, please consider donating to my paypal: 
- https://www.paypal.com/donate/?hosted_button_id=ZJQD2YEQEFVHY
- ![myqrcode](https://user-images.githubusercontent.com/46506059/172424461-c195174a-6450-4b84-ac25-e8632d49fceb.png)



WARNING
------

**USE OF ANY CODE IN THIS REPOSITORY IS AT YOUR OWN RISK.  See Waiver Below**

**I AM NOT A SOFTWARE ENGINEER AND HAVE NO FORMAL TRAINING OR EXPERIENCE WITH ANY OF THIS.  THERE ARE BUGS AND ERRORS IN THIS CODE WHICH IS AT BEST ALPHA QUALITY SOFTWARE AND SHOULD ONLY BE USED FOR RESEARCH PURPOSES. THIS IS NOT A PRODUCT. YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS. NO WARRANTY EXPRESSED OR IMPLIED.**

**You must keep your eyes on the road at all times and be ready to take control of the car at any point.**


---

- [WARNING](#warning)
- [What is openpilot?](#what-is-openpilot)
- [What is in this fork?](#what-is-in-this-fork)
- [Will you add something?](#will-you-add-something)
- [Distance Profiles](#distance-profiles)
- [Distance Profile Cost Adjustments](#distance-profile-cost-adjustments)
- [Improve Acceleration from Stop](#improve-acceleration-from-stop)
- [Development Process](#development-process)
- [Licensing](#licensing)
- [WAIVER](#waiver)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system.

What is in this fork?
------
This is my personal fork of openpilot that includes modifications that I want and nothing else.  I drive a __2019 Rav4 Hybrid (TSS2)__ most of the modifications were designed for my vehicle and my personal taste and may not work on other vehicles.

I am publishing this work to help others understand how openpilot works.  As such, __I am NOT providing support for installation or troubleshooting.__  If you are looking for a supported fork, I recommend [Shane's Stock Additions](https://github.com/sshane/openpilot) fork.  It contains many of the same features.

Currently this fork contains the following modifications:
* 3 Distance profiles that can be toggled using the distance follow button on the steering wheel.  [Feature Toyota Distance Button](https://github.com/krkeegan/openpilot/tree/feature_toyota_distance_btn)
  * These profiles are described below.
  * There are no on screen messages regarding the distance profile selected, only the icons on the vehicle's HUD.
* Toggle disengage on gas from the settings->toggles screen [Feature Disengage on Gas](https://github.com/krkeegan/openpilot/tree/feature_disengage_on_gas)
* Updates are permanently disabled and must be performed using `ssh` and `git`. [Feature Disable Updates](https://github.com/krkeegan/openpilot/tree/feature_disable_updates_testing_msg)
* Raw toggle added back in to enable automatic log uploading. [Feature Raw Logs](https://github.com/krkeegan/openpilot/tree/feature_raw_logs_upload)
* Lower volume to 70% [Feature Lower Volume](https://github.com/krkeegan/openpilot/tree/feature_lower_volume)
* Improve the starting acceleration off the line.  See below. [Feature Slow Start](https://github.com/krkeegan/openpilot/tree/feature_fix_slow_start)
* Specific tuning for my vehicle and my tastes. [Feature Toyota Tune](https://github.com/krkeegan/openpilot/tree/feature_toyota_tune)
  * Tweaks deal with the laggy acceleration response on toyotas
  * Higher acceleration limit at low speed, lower limit at high speed
  * Decrease `a_change_cost` timescale to be closer to toyota lag values
* Other features from future versions of Openpilot as I see fit

Previous Features that have been Merged into Comma Repo:
* Display blue barriers when openpilot is engaged.
* Use wide camera for light sensor, better night performance.

Will you add something?
---
You can certainly ask, but the criteria for adding it is:

* Is it something I would use
* Is it something I consider safe?  I am pretty cautious.
* Is it easy to maintain?  I don't want this to be a chore to maintain.

If the answer to any of those is __no__, then I probably won't add it.

Distance Profiles
---
I extrapolated out the speeds, so that the distance is set about every ~5mph.  This just makes it easier to edit.

* Stock - Unchanged, this uses the default settings in openpilot with follow time of 1.45s.
* Relaxed - This is set to 1.25s across the board.
* Traffic - At low speeds this is set to 1.25s and gradually drops down to 1.05s at freeway speeds.

Distance Profile Cost Adjustments
---
As the following distance decreases I have decreased the `jerk` and `a_change` costs and increased the `danger` cost.  This is to be expected, as the follow distance decreases the allowable rate of change in deceleration has to increase, this will make the acceleration more jerky.  Below is an example of this, as you can see the relaxed and traffic profiles start off following the lead closer, and then when the lead starts to slow down, they increase their rate of deceleration much faster:

![deceleration compared](https://user-images.githubusercontent.com/3046315/148848058-01d3b410-79c2-409a-ab5c-336f21ef8fd9.png)

Improve Acceleration from Stop
---
The new MPC introduced in 8.10 has a much slower acceleration profile from a standstill.  This is caused by many things, but is largely driven by the `a_change_cost` which was introduced as part of the lag compensation.  As you can see below, the rate of acceleration significantly lags behind 8.9:

![8.9 v 8.12](https://user-images.githubusercontent.com/3046315/148848373-7737a46e-a547-48f2-88ec-c2861d42e1ee.png)

I have made adjustments to the `jerk` and `a_change` costs to allow for a faster change in acceleration at low speeds, this affects acceleration only and will not alter braking.  I also reduced the `STOP_DISTANCE` dynamically so that when the lead car pulls away, the MPC is told to maintain a closer distance, this helps cause the an earlier initial acceleration request.  The changes result in the improved profile seen below:

![8.12 v KRK](https://user-images.githubusercontent.com/3046315/148849415-2212361b-4bde-43c2-8f12-bbcdf6d833dd.png)

Development Process
---

The comma.ai repository is a [complicated beast](https://blog.comma.ai/a-2020-theme-externalization/).  The releases are are compiled stripped down repositories.  I prefer to base my daily driver fork `Rav4-TSS2` off of the `commaai/devel` branch.  However, this branch lacks the suite of automated tests.  So I create my development branches off of `commaai/master`.  I try to pick the commit that is closest to the release version and make a branch `master-x.xx` with x.xx being the release version number.

To make merging with future versions of Openpilot easier, I try to keep each "feature" on its own branch. My branches that start with `feature_` contain additional features or tweaks that have not been upstreamed into `commaai/master`.  My branches that start with `future_` contain commits that have been developed by `commaai` but were not included in the last release and are likely to be included in the next release.  Branches prefixed with `research_` are test branches that are likely used for running simulations and may not be designed for use in the real-world.

Once I am satisfied with a branch and it has passed the automated tests, or failed for understandable reasons, I cherry-pick the commits into `Rav4-TSS2`.  Sometimed I may create a `Rav4-TSS2-____` branch if I want to test changes in the vehicle before commiting them to my daily driver branch.

It is all a mess, and it would make working off of my branches difficult, but since this is primarily for personal use, it works for me.

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

WAIVER
-----

Any user of this software ("User") and anyone claiming on User's behalf releases and forever discharges the "Author" of this fork and its directors, officers, employees, agents, stockholders, affiliates, subcontractors, software contributors, and customers from any and all claims, liabilities, obligations, promises, agreements, disputes, demands, damages, causes of action of any nature and kind, known or unknown, which User has or ever had or may in the future have against Author or any of the related parties arising our of or relating to the use of any software in this Repository.

User shall indemnify and hold harmless Author and its directors, officers, employees, agents, stockholders, affiliates, subcontractors, software contributors, and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

