# Tightly-coupled Visual/Inertial/Map Integration with Observability Analysis for Reliable Localization of Intelligent Vehicles

## Abstract

Reliable and cost-effective localization is of great
importance for the realization of intelligent vehicles (IV) in
complex scenes. The visual-inertial odometry (VIO) can provide
high-frequency position estimation over time but is subjected to
drift over time. Recently developed map-aided VIO opens a new
window for drift-free visual localization but the existing loosely
coupled integration fails to fully explore the complementariness
of all the raw measurements. Moreover, the observability of the
existing map-aided VIO is still to be investigated, which is of
great importance for the safety-critical IV. To fill these gaps,
in this article, we propose a factor graph-based state estimator
that tightly couples a 3D lightweight prior line map with a VIO
system and rigorous observability analysis. In particular, for the
cross-modality matching between 3D prior maps and 2D images,
we first utilize the geometric line structure coexisting in the 3D
map and 2D image to build the line feature association model.
More importantly, an efficient line-tracking strategy is designed
to reject the potential line feature-matching outliers. Meanwhile,
a new line feature-based cost model is proposed as a constraint
for factor graph optimization with proof of the rationality behind
this model. Moreover, we also analyze the observability of the
proposed prior line feature-aided VIO system for the first time,
the result shows that x, y, and z three global translations are
observable and the system only has one unobservable direction
theoretically, i.e. the yaw angle around the gravity vector. The
proposed system is evaluated on both simulation outdoor and
real-world indoor environments, and the results demonstrate the
effectiveness of our methods. To benefit the research community,
we open-sourced the dataset with detailed line labeling by
https://github.com/ZHENGXi-git/TC-VIML.
