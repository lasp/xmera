Executive Summary
-----------------

The regions of interest module processes an array of detected regions from image processing and identifies the
primary region of interest representing the target object. Each input region is characterized by its center
coordinates, dimensions, pixel count, and center of brightness. The module employs a multi-stage algorithm to
select or merge regions based on size, spatial proximity, and optional windowing constraints.

The algorithm first applies spatial windowing to restrict the search area, then orders regions by pixel count as
a proxy for brightness or significance. When multiple regions are detected in close proximity, the module computes
a weighted barycenter and merges them into a composite region, addressing scenarios where a single object is
fragmented across multiple detections due to saturation or image processing artifacts.

This module is designed for visual object tracking applications including star tracking, orbital debris detection,
and autonomous spacecraft navigation systems where robust target identification is required despite varying image
quality and detection fragmentation.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the
user from python. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.


.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - regionOutMsg
      - :ref:`RegionOfInterestMsgPayload`
      - Output message containing the identified region of interest with its center of brightness, region center,
        size, number of pixels, and time tag.
    * - roisInMsg
      - :ref:`RegionsIdentifiedMsgPayload`
      - Input message containing an array of detected regions from image processing, each with center coordinates,
        dimensions, pixel count, and center of brightness.

Detailed Module Description
---------------------------

Algorithm Overview
^^^^^^^^^^^^^^^^^^

The region identification algorithm implements a five-stage processing pipeline that transforms multiple detected
regions into a single identified target:

1. **Spatial Windowing**: Applies optional rectangular masking to filter regions based on their center of brightness position relative to user-defined bounds
2. **Region Ordering**: Sorts valid regions by descending pixel count, prioritizing brighter or larger detections
3. **Barycentric Analysis**: Computes the pixel-weighted center of mass across all valid regions
4. **Proximity Classification**: Identifies regions within a specified separation distance from the barycenter
5. **Target Selection**: Selects the largest individual region or merges multiple proximate regions based on proximity analysis

This pipeline enables robust target identification in scenarios where detection algorithms may fragment a single
physical object into multiple regions, while maintaining the ability to discriminate between genuinely distinct
objects based on spatial separation.

Region Windowing
^^^^^^^^^^^^^^^^

Spatial windowing provides a mechanism to constrain the search region to a rectangular subregion of the image
frame. This capability is essential for applications where the approximate target location is known a priori,
enabling computational efficiency and rejection of spurious detections outside the region of interest.

The window is defined by its center point :math:`p_{\mathrm{center}}` and dimensions :math:`(w, h)`. The
algorithm computes the window boundaries as:

.. math::

    p_{\mathrm{topLeft}} &= p_{\mathrm{center}} - \frac{1}{2}\left[ w , h \right]^T \\
    p_{\mathrm{bottomRight}} &= p_{\mathrm{center}} + \frac{1}{2}\left[ w , h \right]^T

A region is retained for further processing if and only if its center of brightness :math:`p_{\mathrm{cob}}`
satisfies the strict inequality conditions:

.. math::

    p_{\mathrm{topLeft,x}} < p_{\mathrm{cob,x}} < p_{\mathrm{bottomRight,x}} \\
    p_{\mathrm{topLeft,y}} < p_{\mathrm{cob,y}} < p_{\mathrm{bottomRight,y}}

Note that boundary pixels are excluded from the valid region. When no window is explicitly configured (center
is zero or dimensions are zero), the algorithm defaults to considering the entire image frame, effectively
disabling spatial filtering.

Region Ordering and Selection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Following spatial windowing, valid regions are sorted in descending order by pixel count. This ordering assumes
that pixel count serves as a reliable proxy for target brightness or significance, with larger detected regions
corresponding to more prominent objects in the scene.

Prior to sorting, regions are filtered against a minimum detection threshold to eliminate noise and spurious
detections. The set of valid regions is defined as:

.. math::

    \mathcal{R}_{\mathrm{valid}} = \{r \in \mathcal{R} \mid N_{\mathrm{pixels}}(r) > N_{\mathrm{min}}\}

where :math:`\mathcal{R}` denotes all input regions and :math:`N_{\mathrm{min}}` represents the configurable
minimum detection size parameter. Regions failing to exceed this threshold are excluded from all subsequent
processing stages.

Barycentric Merging
^^^^^^^^^^^^^^^^^^^

The algorithm employs a barycentric approach to identify and merge regions that likely represent fragments of a
single physical object. This method computes the pixel-weighted center of mass across all valid regions,
providing a robust estimate of the true target center even when detection fragmentation occurs.

The barycenter position is computed as:

.. math::

    p_{\mathrm{barycenter}} = \frac{\sum_{r \in \mathcal{R}_{\mathrm{valid}}} N_{\mathrm{pixels}}(r) \cdot p_{\mathrm{cob}}(r)}{\sum_{r \in \mathcal{R}_{\mathrm{valid}}} N_{\mathrm{pixels}}(r)}

where :math:`p_{\mathrm{cob}}(r)` denotes the center of brightness of region :math:`r`, and :math:`N_{\mathrm{pixels}}(r)`
provides the weighting factor for each region's contribution to the collective center.

The algorithm then quantifies the spatial clustering of regions relative to this barycenter by computing:

.. math::

    N_{\mathrm{close}} = |\{r \in \mathcal{R}_{\mathrm{valid}} \mid \|p_{\mathrm{cob}}(r) - p_{\mathrm{barycenter}}\| < d_{\mathrm{max}}\}|

where :math:`d_{\mathrm{max}}` is the maximum separation parameter and :math:`\|\cdot\|` denotes the Euclidean
norm. This count determines the subsequent merging behavior:

.. list-table:: Merging Decision Logic
    :widths: 30 70
    :header-rows: 1

    * - Condition
      - Action
    * - :math:`N_{\mathrm{close}} < 2`
      - Return the largest region without modification, as only one significant region exists
    * - :math:`N_{\mathrm{close}} \geq 2`
      - Merge all regions into a composite region centered at the barycenter

This approach provides robustness against detection fragmentation while maintaining the ability to discriminate
between genuinely distinct objects separated by distances exceeding :math:`d_{\mathrm{max}}`.

Merged Region Properties
^^^^^^^^^^^^^^^^^^^^^^^^^

When the proximity analysis indicates multiple closely-spaced regions (:math:`N_{\mathrm{close}} \geq 2`), the
algorithm constructs a composite region with properties derived from the constituent detections. The merged region
inherits the following characteristics:

**Spatial Location:**

Both the center of brightness and geometric center are assigned to the computed barycenter position:

.. math::

    p_{\mathrm{ROI,cob}} = p_{\mathrm{ROI,center}} = p_{\mathrm{barycenter}}

This ensures the merged region is centered at the pixel-weighted centroid of all contributing detections.

**Pixel Count:**

The total pixel count represents the sum across all valid input regions:

.. math::

    N_{\mathrm{ROI,pixels}} = \sum_{r \in \mathcal{R}_{\mathrm{valid}}} N_{\mathrm{pixels}}(r)

This aggregation preserves the total detected brightness information.

**Bounding Dimensions:**

The region size is defined as the side length of a square inscribed within a circle of radius :math:`d_{\mathrm{max}}`:

.. math::

    w_{\mathrm{ROI}} = h_{\mathrm{ROI}} = \left\lfloor \frac{d_{\mathrm{max}}}{\sqrt{2}} \right\rfloor

This sizing strategy ensures the bounding box encompasses all regions within the maximum separation distance from
the barycenter. If some bright pixels that were not part of the center of brightness computation are included in this
new window, they will be included in downstream center of brightness computations.

Algorithm Behavior and Parameter Selection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Null Detection Handling:**

When all input regions have pixel counts at or below the minimum detection threshold :math:`N_{\mathrm{min}}`,
the algorithm returns an empty region with zero pixel count. This provides an unambiguous indication to downstream
processing modules that no valid target was identified in the current frame.

**Single Region Case:**

When exactly one region exceeds the detection threshold, it is returned directly without barycentric computation.
This optimization reduces computational overhead while maintaining algorithmic correctness for the common
single-target scenario.

**Windowing Priority:**

Spatial windowing takes precedence over brightness-based selection. Regions with centers of brightness outside
the defined window bounds are excluded from all processing stages, regardless of their pixel count. This behavior
enables deterministic target selection within a constrained search region, supporting applications where spatial
constraints are more critical than brightness maximization.

**Maximum Separation Parameter Selection:**

The :math:`d_{\mathrm{max}}` parameter controls the spatial scale at which regions are considered potentially
related. Appropriate values depend on the expected target size, camera resolution, and anticipated fragmentation
characteristics.

User Guide
----------

This section provides implementation guidance for integrating the Regions of Interest module into a flight
software simulation. The following steps demonstrate the standard configuration workflow in Python.

#. Import the regionsOfInterest class:

.. code-block:: python

    from Basilisk.fswAlgorithms import regionsOfInterest

#. Create an instantiation of the RegionsOfInterest module::

    roiModule = regionsOfInterest.RegionsOfInterest()
    roiModule.ModelTag = "roiModule"

#. Configure the maximum separation distance for region merging. This parameter defines the spatial threshold
   for determining whether multiple detected regions should be merged into a single composite target:

.. code-block:: python

    roiModule.setMaxRoiSeparation(100)  # pixels

   Regions with centers of brightness separated by less than this distance from the barycenter may be merged,
   depending on the proximity analysis outcome.

#. (Optional) Configure spatial windowing to constrain the search region:

.. code-block:: python

    import numpy as np
    windowCenter = np.array([512, 384], dtype=int)  # pixels
    roiModule.setWindowCenter(windowCenter)
    roiModule.setWindowSize(640, 480)  # width, height in pixels
    roiModule.setImageSize(1250, 1000)  # width, height in pixels

   Windowing restricts processing to a rectangular subregion of the image frame. Regions with centers of
   brightness outside this window are excluded regardless of their brightness. Omit this configuration to
   process the entire image frame.

#. Subscribe to the regions input message produced by an upstream image processing module::

    roiModule.roisInMsg.subscribeTo(regionsInMsg)

   The input message must contain an array of detected regions, each characterized by center coordinates,
   dimensions, pixel count, and center of brightness position.

#. Register the module with the simulation task manager::

    sim.AddModelToTask(taskName, roiModule)

#. Configure output message recording to capture the identified region of interest::

    roiLog = roiModule.regionOutMsg.recorder()
    sim.AddModelToTask(taskName, roiLog)

   The output message contains the identified ROI properties including center of brightness, geometric center,
   bounding dimensions, total pixel count, and time tag.

Advanced Configuration
^^^^^^^^^^^^^^^^^^^^^^

The following additional configuration parameters are available for specialized applications:

**Camera Identifier Assignment:**

For multi-camera systems, assign a unique camera identifier to facilitate downstream correlation::

    roiModule.setCameraId(1)

**Minimum Detection Threshold:**

Configure the minimum pixel count threshold for valid region detection. Regions below this threshold are
excluded as noise or spurious detections:

.. code-block:: python

    roiModule.setMinimumDetectionSize(10)  # pixels

This parameter should be tuned based on the expected signal-to-noise characteristics of the imaging system
and the anticipated size of valid targets. Conservative (larger) values reduce false positive detections at
the cost of potentially missing dim or small targets.

Module Verification
-------------------

A comprehensive verification suite is provided in the ``tests`` directory to validate module functionality
across a range of operating conditions:

.. list-table:: Test Suite Components
    :widths: 40 60
    :header-rows: 1

    * - Test File
      - Coverage
    * - ``test_regionsOfInterest.cpp``
      - Unit tests for all algorithm components and edge cases
    * - ``test_regionsOfInterest_fuzz.cpp``
      - Fuzz testing for robustness validation with randomized inputs
    * - ``test_regionsOfInterest.py``
      - Integration tests within full simulation framework

Module Assumptions and Limitations
-----------------------------------

**Algorithm Assumptions:**

The module implementation relies on the following assumptions regarding input data and operating conditions:

#. Region pixel count provides a monotonic relationship with target brightness or significance within each frame
#. Input region centers of brightness are expressed in a consistent image coordinate frame

**Operational Limitations:**

#. **Input Capacity:** Processing is constrained to ``MAX_NUMBER_REGIONS`` input regions per execution cycle. Excess detections are not processed.
#. **Temporal Independence:** Each module execution is stateless with respect to previous frames. No temporal correlation or tracking is performed across successive updates.
#. **Window Validation:** Spatial window boundary checking assumes standard image dimensions. User-specified windows extending beyond image bounds will trigger validation errors during initialization.
