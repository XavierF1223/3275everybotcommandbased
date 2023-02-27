// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;


public class OrchestraSub extends SubsystemBase {
  Orchestra _orchestra;

  TalonFX []_instruments = {
    new TalonFX(DriveConstants.FL),
    new TalonFX(DriveConstants.FR),
    new TalonFX(DriveConstants.RL),
    new TalonFX(DriveConstants.RR),
    new TalonFX(ArmConstants.ArmMotorLeft),
    new TalonFX(ArmConstants.ArmMotorRight)};

  public String []_songs = new String[]{
    "doom.chrp",
    "feelgood.chrp",
    "heartshapedbox.chrp",
    "nationalanthem.chrp",
    "stayinalive.chrp"
  };
  String _songSelection = "";
  int _song = 0;
  int _timeToPlayLoops = 0;

  /** Creates a new Orchestra. */
  public OrchestraSub() {
    ArrayList <TalonFX> instruments = new ArrayList<TalonFX>();
    for (int i = 0; i < _instruments.length; ++i){
      instruments.add(_instruments[i]);
    }
    _orchestra = new Orchestra(instruments);

  }

  public void LoadMusicSelection(String song)
  {
      /* increment song selection */
      _songSelection = song;
      for (int i = 0; i < _songs.length; ++i){
        if (_songSelection == _songs[i]){
            _song = i;
          }
      }

      /* load the chirp file */
      _orchestra.loadMusic(_songs[_song]); 

      /* print to console */
      System.out.println("Song selected is: " + _songs[_song] + ".  Press left/right on d-pad to change.");
      
      /* schedule a play request, after a delay.  
          This gives the Orchestra service time to parse chirp file.
          If play() is called immedietely after, you may get an invalid action error code. */
      _timeToPlayLoops = 10;
  }

  public void play(){
    _orchestra.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
}
