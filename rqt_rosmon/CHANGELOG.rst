^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_rosmon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2020-05-28)
------------------

2.2.1 (2019-11-08)
------------------

2.2.0 (2019-10-29)
------------------
* rqt_rosmon: use std::sort() instead of qSort()
  qSort() is deprecated in newer Qt versions.
* rqt_rosmon: node_model: fix dataChanged() column indices
  Column indices are end-inclusive, so we should give COL_COUNT-1. This
  fixes an issue where the table view would not update on new data.
* Contributors: Max Schwarz

2.1.1 (2019-07-09)
------------------

2.1.0 (2019-06-25)
------------------
* handle namespaces in service calls correctly, PR #80
* show node namespaces, PR #78
* add dependencies for catkin_make builds, PR #75
* Contributors: Artur Miller, Adrien BARRAL, David Walsh, Eric Fang, Max Schwarz

2.0.2 (2019-03-12)
------------------

2.0.1 (2019-03-12)
------------------
* rqt_rosmon: declare pluginlib dependency in package.xml
* Contributors: Max Schwarz

2.0.0 (2019-03-11)
------------------
* split into separate packages for core, GUI, messages, PR #72
* Contributors: Max Schwarz

1.0.10 (2018-10-29)
-------------------

1.0.9 (2018-09-20)
------------------

1.0.8 (2018-08-07)
------------------

1.0.7 (2018-05-27)
------------------

1.0.6 (2018-05-26)
------------------

1.0.5 (2018-05-25)
------------------

1.0.4 (2018-05-24)
------------------

1.0.3 (2018-05-05)
------------------

1.0.2 (2018-04-24)
------------------

1.0.1 (2018-04-13 18:55)
------------------------

1.0.0 (2018-04-13 12:15)
------------------------
