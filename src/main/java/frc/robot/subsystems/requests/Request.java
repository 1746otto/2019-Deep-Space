package frc.robot.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

public abstract class Request {
  public abstract void act();

  public boolean isFinished() {
    return true;
  }

  public List<Prerequisite> prerequisites = new ArrayList<>();

  public void withPrerequisites(List<Prerequisite> prereqs) {
    for (Prerequisite req : prerequisites) {
      prerequisites.add(req);
    }
  }

  public boolean allowed() {
    boolean reqsMet = true;
    for (Prerequisite req : prerequisites) {
      reqsMet &= req.met();
    }
    return reqsMet;
  }
}