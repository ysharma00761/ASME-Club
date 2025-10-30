// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::FULL4WIRE, 4, 5, 6, 7);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 8, 9, 10, 11);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

void setup() {
  Serial.begin(9600);

  // Configure each stepper
  stepper1.setMaxSpeed(2500);
  stepper1.setAcceleration(1800);
  stepper2.setMaxSpeed(2500);
  stepper2.setAcceleration(3000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

 stepper1.moveTo(1000);
 stepper2.moveTo(1000);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper1.distanceToGo() == 0)
      stepper1.moveTo(-stepper1.currentPosition());

    if (stepper2.distanceToGo() == 0)
      stepper2.moveTo(-stepper2.currentPosition());

    stepper1.run();
    stepper2.run();

}
