package org.ietr.preesm.evaluator.mathModels;

import org.eclipse.core.runtime.Plugin;
import org.eclipse.ui.IStartup;
import org.ietr.preesm.evaluator.Activator;
import org.ietr.preesm.schedule.PeriodicScheduler_SDF;
import org.osgi.framework.BundleContext;

/**
 * 
 * @author hderoui
 *
 */
public class MathsModelsPlugin extends Plugin implements IStartup {

  @Override
  public void start(BundleContext context) throws Exception {
    Activator.solverMethodRegistry.put(PeriodicScheduler_SDF.Method.LinearProgram_Gurobi, new PeriodicScheduleModel_Gurobi());
    super.start(context);
  }

  @Override
  public void stop(BundleContext context) throws Exception {
    super.stop(context);
    Activator.solverMethodRegistry.remove(PeriodicScheduler_SDF.Method.LinearProgram_Gurobi);
  }

  @Override
  public void earlyStartup() {
    Activator.solverMethodRegistry.put(PeriodicScheduler_SDF.Method.LinearProgram_Gurobi, new PeriodicScheduleModel_Gurobi());
  }

}
