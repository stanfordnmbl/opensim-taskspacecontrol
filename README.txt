
This is currently developed as a plugin to OpenSim. In the future, the plumbing
of the controller will be implemented in Simbody, and the OpenSim code base
will include the necessary wrapping classes (for serialization, etc.).

TODO
====

* Change OSIMTASKSPACE_API to OSIMSIMULATION_API?
* The methods take in references to the output, instead of returning Matrices,
    * etc.


Notes
=====

Simbody
-------
There is a plan to 'implement' task space in Simbody.


Decisions
=========

CompositeTask or PriorityLevel?
-------------------------------
PriorityLevel is easier; I'll do that for now.

Using a namespace, TaskSpace
----------------------------
There's already lots of classes in OpenSim that smell like operational space /
task space classes (e.g., CMC_Task, TrackingTask, ...). The controller requires
a number of classes. I could make all of those nested within a
TaskSpaceController class, or I could prepend their names with TaskSpace. The
former creates issues for wrapping, and the latter makes very long class names,
and is a weaker form of creating a namespace.

Why 'TaskSpace' instead of, e.g., OpSpace, OSC, ...?
----------------------------------------------------
I wanted a short name that did not include any abbreviations or acronyms.
I'm open to suggestions here, though, as I am not sure TaskSpace is really
the right term.

