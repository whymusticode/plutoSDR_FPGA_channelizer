# Sourced from the stock system_project.tcl before adi_project_run (scripts/pluto-fw).
adi_project_files pluto [list "$::env(PLUTO_PROJECT_DIR)/hdl/constr.xdc"]
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
