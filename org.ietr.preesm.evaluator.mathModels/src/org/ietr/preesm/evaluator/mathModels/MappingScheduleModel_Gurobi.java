package org.ietr.preesm.evaluator.mathModels;

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;
import java.util.HashMap;
import java.util.Map.Entry;
import org.ietr.dftools.algorithm.model.sdf.SDFAbstractVertex;
import org.ietr.dftools.algorithm.model.sdf.SDFEdge;
import org.ietr.dftools.algorithm.model.sdf.SDFGraph;

/**
 * @author hderoui
 *
 */
public class MappingScheduleModel_Gurobi {
  public HashMap<String, HashMap<Integer, HashMap<Integer, GRBVar>>> varSet_X_a_c_t;
  public HashMap<String, HashMap<Integer, GRBVar>>                   varSet_S_a_t;
  public HashMap<String, HashMap<Integer, GRBVar>>                   varSet_R_a_c;

  /**
   * compute the optimal solution for the mapping and scheduling problem
   * 
   * @param SDF
   *          graph
   * @param nbCores
   *          number of resources
   * @param nbTimeSlot
   *          time horizon
   */
  public void computeNormalizedPeriod(SDFGraph SDF, int nbCores, int nbTimeSlot) {
    // Stopwatch timerPerSche = new Stopwatch();
    // timerPerSche.start();
    varSet_X_a_c_t = new HashMap<>(SDF.vertexSet().size());
    varSet_S_a_t = new HashMap<>(SDF.vertexSet().size());
    varSet_R_a_c = new HashMap<>(SDF.vertexSet().size());

    try {
      // ----- create the Gurobi model ---------------------------------
      GRBEnv env = new GRBEnv();
      GRBModel model = new GRBModel(env);
      model.set(GRB.StringAttr.ModelName, "OptimalMappingScheduling");

      // ----- Variables ---------------------------------------------
      // X_a_c_ts variables
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        varSet_X_a_c_t.put(actor.getName(), new HashMap<>());
        for (int c = 0; c < nbCores; c++) {
          varSet_X_a_c_t.get(actor.getName()).put(c, new HashMap<>());
          for (int t = 0; t < nbTimeSlot; t++) {
            varSet_X_a_c_t.get(actor.getName()).get(c).put(t, model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + actor.getName() + "_c" + c + "_ts" + t));
          }
        }
      }

      // S_a_t variables
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        varSet_S_a_t.put(actor.getName(), new HashMap<>());
        int Smax = nbTimeSlot - ((int) actor.getPropertyBean().getValue("duration")) + 1;
        for (int t = 0; t <= Smax; t++) {
          varSet_S_a_t.get(actor.getName()).put(t, model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "S_" + actor.getName() + "_ts" + t));
        }
      }

      // R_a_c varaibles
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        varSet_R_a_c.put(actor.getName(), new HashMap<>());
        for (int c = 0; c < nbCores; c++) {
          varSet_R_a_c.get(actor.getName()).put(c, model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "R_" + actor.getName() + "_c" + c));
        }
      }

      model.update();

      // ----- Objectif function -------------------------------------
      GRBLinExpr expr = new GRBLinExpr();
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        for (Entry<Integer, GRBVar> entry : varSet_S_a_t.get(actor.getName()).entrySet()) {
          expr.addTerm(entry.getKey(), entry.getValue());
        }
      }
      model.setObjective(expr, GRB.MINIMIZE);

      // ----- Constraints ------------------------------------------
      // StartDate constraint: fix(a), var(c,t), Sum(X_a_c_t) >= l.St
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        double dur = (Double) actor.getPropertyBean().getValue("duration");
        for (Entry<Integer, GRBVar> e : varSet_S_a_t.get(actor.getName()).entrySet()) {
          expr = new GRBLinExpr();
          for (int t = e.getKey(); t < e.getKey() + dur; t++) {
            for (int c = 0; c < nbCores; c++) {
              expr.addTerm(1.0, varSet_X_a_c_t.get(actor.getName()).get(c).get(t));
            }
          }
          expr.addTerm(-1 * dur, e.getValue());
          model.addConstr(expr, GRB.GREATER_EQUAL, 0.0, "SC_" + actor.getName() + "_s" + e.getKey());
        }
      }

      // Resource constraint: fix(a,c), var(t), Sum(X_a_c_t) <= (=) l.R
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        double dur = (Double) actor.getPropertyBean().getValue("duration");
        for (int c = 0; c < nbCores; c++) {
          expr = new GRBLinExpr();
          for (int t = 0; t < nbTimeSlot; t++) {
            expr.addTerm(1.0, varSet_X_a_c_t.get(actor.getName()).get(c).get(t));
          }
          expr.addTerm(-1 * dur, varSet_R_a_c.get(actor.getName()).get(c));
          model.addConstr(expr, GRB.LESS_EQUAL, 0.0, "RC_" + actor.getName() + "_c" + c);
        }
      }

      // duration constraint: fix(a), var(c,t), Sum(X_a_c_t) = l ===> can be ignored by setting RC = l*R
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        double dur = (Double) actor.getPropertyBean().getValue("duration");
        expr = new GRBLinExpr();
        for (int c = 0; c < nbCores; c++) {
          for (int t = 0; t < nbTimeSlot; t++) {
            expr.addTerm(1.0, varSet_X_a_c_t.get(actor.getName()).get(c).get(t));
          }
        }
        model.addConstr(expr, GRB.EQUAL, dur, "DurC_" + actor.getName());
      }

      // only one start date need to be activated
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        expr = new GRBLinExpr();
        for (Entry<Integer, GRBVar> e : varSet_S_a_t.get(actor.getName()).entrySet()) {
          expr.addTerm(1.0, e.getValue());
        }
        model.addConstr(expr, GRB.EQUAL, 1.0, "OneExC_" + actor.getName());
      }

      // only one resource need to be used
      for (SDFAbstractVertex actor : SDF.vertexSet()) {
        expr = new GRBLinExpr();
        for (Entry<Integer, GRBVar> e : varSet_R_a_c.get(actor.getName()).entrySet()) {
          expr.addTerm(1.0, e.getValue());
        }
        model.addConstr(expr, GRB.EQUAL, 1.0, "OneResC_" + actor.getName());
      }

      // each resource execute one actor or less at a time
      for (int c = 0; c < nbCores; c++) {
        for (int t = 0; t < nbTimeSlot; t++) {
          expr = new GRBLinExpr();
          for (SDFAbstractVertex actor : SDF.vertexSet()) {
            expr.addTerm(1.0, varSet_X_a_c_t.get(actor.getName()).get(c).get(t));
          }
          model.addConstr(expr, GRB.LESS_EQUAL, 1.0, "oneActorPerTSC_" + c + "_t" + t);
        }
      }

      // precedence constraints
      for (SDFEdge edge : SDF.edgeSet()) {
        expr = new GRBLinExpr();
        for (Entry<Integer, GRBVar> entry : varSet_S_a_t.get(edge.getTarget().getName()).entrySet()) {
          expr.addTerm(entry.getKey(), entry.getValue());
        }

        for (Entry<Integer, GRBVar> entry : varSet_S_a_t.get(edge.getSource().getName()).entrySet()) {
          expr.addTerm(-1 * entry.getKey(), entry.getValue());
        }

        model.addConstr(expr, GRB.GREATER_EQUAL, (Double) edge.getSource().getPropertyBean().getValue("duration"),
            "precC_e_from" + edge.getSource().getName() + "_to_" + edge.getTarget().getName());
      }

      // ----- solve the problem -------------------------------------
      model.optimize();

      if (model.get(GRB.IntAttr.Status) == GRB.Status.INFEASIBLE) {
        System.err.println("Model is infeasible or unbounded");
      } else {
        if (model.get(GRB.DoubleAttr.ObjVal) >= 0) {
          System.out.println("sum start Date = " + model.get(GRB.DoubleAttr.ObjVal));
        }
        // print time line for each actor per resource if is activated

        // print the latency
        // loop the actor get the start date + dur
        // the max value is the latency
      }
      model.dispose();
      env.dispose();
    } catch (

    GRBException e) {
      e.printStackTrace();
    }

    // timerPerSche.stop();
    // System.out.println("SDF Graph Scheduled in " + timerPerSche.toString());

  }
}
