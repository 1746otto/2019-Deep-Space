package frc.robot.lib.team254.geometry;

import frc.robot.lib.team254.util.CSVWritable;
import frc.robot.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
