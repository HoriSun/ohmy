#ifndef ecMkControlWidget_H_
#define ecMkControlWidget_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecMkControlWidget.h
/// @class EcMkControlWidget
/// @brief MK robot control widget
//
//------------------------------------------------------------------------------
#include <QtWidgets/QWidget>

#include <control/ecManipEndEffectorPlace.h>
#include <manipulator/ecGravitationalTorqueTool.h>
#include <manipulator/ecIndManipulator.h>
#include <rtosCommon/ecRtosConstants.h>

#include <boost/shared_ptr.hpp>

#include <QtWidgets/QMessageBox>
#include <QtCore/QTimer>

// forward declarations
namespace Ec { class PluginGUI; }
namespace Ui { class ecMkControlWidget; }
//class EcRtosDataExchange;
class EcRunningAverageFilter;
class mkControlPluginExec;
class manipulationDirectorPlugin;
class QLabel;

class EcMkControlWidget : public QWidget
{
   Q_OBJECT

public:
   /// constructor
   EcMkControlWidget
      (
      Ec::PluginGUI& gui,
      mkControlPluginExec* pMkControlPluginExec,
      manipulationDirectorPlugin* pManipulationDirector
      );

   /// init state
   EcBoolean initState
      (
      );

protected Q_SLOTS:
   void update();

   void onPushButtonInitClicked();
   void onPushButtonResetClicked();
   void onPushButtonEnableClicked();
   void onPushButtonDisableClicked();
   void onPushButtonSimulateClicked();
   void onPushButtonStopSimulationClicked();
   void onPushButtonStopClicked();
   void onPushButtonHomePressed();
   void onPushButtonHomeReleased();
   void onPushButtonDecreaseSpeedFactorClicked();
   void onPushButtonIncreaseSpeedFactorClicked();

   // joint and EE control
   void onPushButtonJointDown0Clicked();
   void onPushButtonJointDown1Clicked();
   void onPushButtonJointDown2Clicked();
   void onPushButtonJointDown3Clicked();
   void onPushButtonJointDown4Clicked();
   void onPushButtonJointDown5Clicked();
   void onPushButtonJointUp0Clicked();
   void onPushButtonJointUp1Clicked();
   void onPushButtonJointUp2Clicked();
   void onPushButtonJointUp3Clicked();
   void onPushButtonJointUp4Clicked();
   void onPushButtonJointUp5Clicked();
   void onPushButtonEeDownXClicked();
   void onPushButtonEeDownYClicked();
   void onPushButtonEeDownZClicked();
   void onPushButtonEeDownRollClicked();
   void onPushButtonEeDownPitchClicked();
   void onPushButtonEeDownYawClicked();
   void onPushButtonEeUpXClicked();
   void onPushButtonEeUpYClicked();
   void onPushButtonEeUpZClicked();
   void onPushButtonEeUpRollClicked();
   void onPushButtonEeUpPitchClicked();
   void onPushButtonEeUpYawClicked();

   void onPushButtonRunScript();
   void onPushButtonStopScript();

protected:
   void updateEePlacement();
   EcReal getStepSize();
   void setDesiredJoint();
   void setDesiredTranslation(const EcVector& vec);
   void setDesiredOrientation(const EcOrientation& ori);
   void disconnectRtos();

protected:
   boost::shared_ptr<Ui::ecMkControlWidget>           m_pUi;
   Ec::PluginGUI&                                     m_PluginGui;
   mkControlPluginExec*                               m_pControlPluginExec;
   manipulationDirectorPlugin*                        m_pManipulationDirector;      ///< the manipulation director plugin

   EcInt32                             m_StateMachineState;
   EcInt32Vector                       m_BrakesStatus;
   EcInt32Vector                       m_AxisFaults;
   EcRtos::Operation                   m_RequestedOperation;
   EcRealVector                        m_JointTorquesInst;
   std::vector<EcRunningAverageFilter> m_RunningAverageFilters;
   EcRealVector                        m_JointTorquesAvg;
   EcU32                               m_EeSetIndex;
   EcRealVector                        m_NominalEeSpeeds;
   EcManipulatorEndEffectorPlacement   m_ManipulatorEePlacement;
   EcEndEffectorPlacement              m_DesiredEePlacement;
   EcRealVector                        m_ActualJointValues;
   mutable EcPositionState             m_PositionState;

   // for updating
   QTimer                              m_GuiUpdateTimer;

   // for display
   typedef std::vector<QLabel*>  QLabelPointerVector;
   QLabelPointerVector                 m_BrakeStatusLabels;
   QLabelPointerVector                 m_AxisFaultLabels;
   QLabelPointerVector                 m_JointTorqueInstLabels;
   QLabelPointerVector                 m_JointTorqueAvgLabels;
   QLabelPointerVector                 m_JointTorqueExpLabels;
   QLabelPointerVector                 m_ActualJointLabels;

   // for torque
   EcIndividualManipulator             m_MkManipulator;
   EcGravitationalTorqueTool           m_GravitationalTorqueTool;
   EcReArray                           m_GravitationalTorques;

   // for emergency stop
   QMessageBox                         m_QMsgBox;
};

#endif // ecMkControlWidget_H_
