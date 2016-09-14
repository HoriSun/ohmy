//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecMkControlWidget.cpp
//
//------------------------------------------------------------------------------
#include "ecMkControlWidget.h"
#include "ui_ecMkControlWidget.h"
#include <viewerCore/ecPluginGUI.h>
#include <plugins/ecIOParams.h>
#include <control/ecVelocityController.h>
#include <control/ecManipEndEffectorPlace.h>
#include <control/ecPosContSystem.h>
#include <rendCore/ecRenderTypes.h>
#include <signalFilter/ecRunningAverageFilter.h>
#include <manipulationDirectorPlugin/manipulationDirectorPlugin.h>
#include <mkControlPluginExec/mkControlPluginExec.h>
#include <motionPlanning/ecMotionScriptParser.h>

//------------------------------------------------------------------------------
EcMkControlWidget::EcMkControlWidget
   (
   Ec::PluginGUI& gui,
   mkControlPluginExec* pMkControlPluginExec,
   manipulationDirectorPlugin* pManipulationDirector
   ) :
QWidget(&gui),
m_pUi(new Ui::ecMkControlWidget),
m_PluginGui(gui),
m_pControlPluginExec(pMkControlPluginExec),
m_pManipulationDirector(pManipulationDirector),
m_StateMachineState(0),
m_RequestedOperation(EcRtos::Operation::STANDBY),
m_JointTorquesInst(EcRtos::MK_NUM_AXES, 0.0),
m_RunningAverageFilters(EcRtos::MK_NUM_AXES, EcRunningAverageFilter(100)),
m_JointTorquesAvg(EcRtos::MK_NUM_AXES, 0.0),
m_EeSetIndex(0),
m_BrakeStatusLabels(EcRtos::MK_NUM_AXES, EcNULL),
m_AxisFaultLabels(EcRtos::MK_NUM_AXES, EcNULL),
m_JointTorqueInstLabels(EcRtos::MK_NUM_AXES, EcNULL),
m_JointTorqueAvgLabels(EcRtos::MK_NUM_AXES, EcNULL),
m_JointTorqueExpLabels(EcRtos::MK_NUM_AXES, EcNULL),
m_ActualJointLabels(EcRtos::MK_NUM_AXES, EcNULL)
{
   m_pUi->setupUi(this);

   m_BrakeStatusLabels[0] = m_pUi->labelJointBrake0;
   m_BrakeStatusLabels[1] = m_pUi->labelJointBrake1;
   m_BrakeStatusLabels[2] = m_pUi->labelJointBrake2;
   m_BrakeStatusLabels[3] = m_pUi->labelJointBrake3;
   m_BrakeStatusLabels[4] = m_pUi->labelJointBrake4;
   m_BrakeStatusLabels[5] = m_pUi->labelJointBrake5;

   m_AxisFaultLabels[0] = m_pUi->labelJointFault0;
   m_AxisFaultLabels[1] = m_pUi->labelJointFault1;
   m_AxisFaultLabels[2] = m_pUi->labelJointFault2;
   m_AxisFaultLabels[3] = m_pUi->labelJointFault3;
   m_AxisFaultLabels[4] = m_pUi->labelJointFault4;
   m_AxisFaultLabels[5] = m_pUi->labelJointFault5;

   m_JointTorqueInstLabels[0] = m_pUi->labelTorqueInst0;
   m_JointTorqueInstLabels[1] = m_pUi->labelTorqueInst1;
   m_JointTorqueInstLabels[2] = m_pUi->labelTorqueInst2;
   m_JointTorqueInstLabels[3] = m_pUi->labelTorqueInst3;
   m_JointTorqueInstLabels[4] = m_pUi->labelTorqueInst4;
   m_JointTorqueInstLabels[5] = m_pUi->labelTorqueInst5;

   m_JointTorqueAvgLabels[0] = m_pUi->labelTorqueAvg0;
   m_JointTorqueAvgLabels[1] = m_pUi->labelTorqueAvg1;
   m_JointTorqueAvgLabels[2] = m_pUi->labelTorqueAvg2;
   m_JointTorqueAvgLabels[3] = m_pUi->labelTorqueAvg3;
   m_JointTorqueAvgLabels[4] = m_pUi->labelTorqueAvg4;
   m_JointTorqueAvgLabels[5] = m_pUi->labelTorqueAvg5;

   m_JointTorqueExpLabels[0] = m_pUi->labelTorqueExp0;
   m_JointTorqueExpLabels[1] = m_pUi->labelTorqueExp1;
   m_JointTorqueExpLabels[2] = m_pUi->labelTorqueExp2;
   m_JointTorqueExpLabels[3] = m_pUi->labelTorqueExp3;
   m_JointTorqueExpLabels[4] = m_pUi->labelTorqueExp4;
   m_JointTorqueExpLabels[5] = m_pUi->labelTorqueExp5;

   m_ActualJointLabels[0] = m_pUi->labelActualJoint0;
   m_ActualJointLabels[1] = m_pUi->labelActualJoint1;
   m_ActualJointLabels[2] = m_pUi->labelActualJoint2;
   m_ActualJointLabels[3] = m_pUi->labelActualJoint3;
   m_ActualJointLabels[4] = m_pUi->labelActualJoint4;
   m_ActualJointLabels[5] = m_pUi->labelActualJoint5;

   connect(m_pUi->pushButtonInit, SIGNAL(clicked()), this, SLOT(onPushButtonInitClicked()));
   connect(m_pUi->pushButtonReset, SIGNAL(clicked()), this, SLOT(onPushButtonResetClicked()));
   connect(m_pUi->pushButtonEnable, SIGNAL(clicked()), this, SLOT(onPushButtonEnableClicked()));
   connect(m_pUi->pushButtonDisable, SIGNAL(clicked()), this, SLOT(onPushButtonDisableClicked()));
   connect(m_pUi->pushButtonSimulate, SIGNAL(clicked()), this, SLOT(onPushButtonSimulateClicked()));
   connect(m_pUi->pushButtonStopSimulation, SIGNAL(clicked()), this, SLOT(onPushButtonStopSimulationClicked()));
   connect(m_pUi->pushButtonStop, SIGNAL(clicked()), this, SLOT(onPushButtonStopClicked()));
   connect(m_pUi->pushButtonHome, SIGNAL(pressed()), this, SLOT(onPushButtonHomePressed()));
   connect(m_pUi->pushButtonHome, SIGNAL(released()), this, SLOT(onPushButtonHomeReleased()));
   connect(m_pUi->pushButtonDecreaseSpeedFactor, SIGNAL(clicked()), this, SLOT(onPushButtonDecreaseSpeedFactorClicked()));
   connect(m_pUi->pushButtonIncreaseSpeedFactor, SIGNAL(clicked()), this, SLOT(onPushButtonIncreaseSpeedFactorClicked()));

   // joint and EE control
   connect(m_pUi->pushButtonJointDown0, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown0Clicked()));
   connect(m_pUi->pushButtonJointDown1, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown1Clicked()));
   connect(m_pUi->pushButtonJointDown2, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown2Clicked()));
   connect(m_pUi->pushButtonJointDown3, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown3Clicked()));
   connect(m_pUi->pushButtonJointDown4, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown4Clicked()));
   connect(m_pUi->pushButtonJointDown5, SIGNAL(clicked()), this, SLOT(onPushButtonJointDown5Clicked()));
   connect(m_pUi->pushButtonJointUp0, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp0Clicked()));
   connect(m_pUi->pushButtonJointUp1, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp1Clicked()));
   connect(m_pUi->pushButtonJointUp2, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp2Clicked()));
   connect(m_pUi->pushButtonJointUp3, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp3Clicked()));
   connect(m_pUi->pushButtonJointUp4, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp4Clicked()));
   connect(m_pUi->pushButtonJointUp5, SIGNAL(clicked()), this, SLOT(onPushButtonJointUp5Clicked()));
   connect(m_pUi->pushButtonEeDownX, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownXClicked()));
   connect(m_pUi->pushButtonEeDownY, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownYClicked()));
   connect(m_pUi->pushButtonEeDownZ, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownZClicked()));
   connect(m_pUi->pushButtonEeDownRoll, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownRollClicked()));
   connect(m_pUi->pushButtonEeDownPitch, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownPitchClicked()));
   connect(m_pUi->pushButtonEeDownYaw, SIGNAL(clicked()), this, SLOT(onPushButtonEeDownYawClicked()));
   connect(m_pUi->pushButtonEeUpX, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpXClicked()));
   connect(m_pUi->pushButtonEeUpY, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpYClicked()));
   connect(m_pUi->pushButtonEeUpZ, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpZClicked()));
   connect(m_pUi->pushButtonEeUpRoll, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpRollClicked()));
   connect(m_pUi->pushButtonEeUpPitch, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpPitchClicked()));
   connect(m_pUi->pushButtonEeUpYaw, SIGNAL(clicked()), this, SLOT(onPushButtonEeUpYawClicked()));

   // script
   connect(m_pUi->pushButtonRunScript, SIGNAL(clicked()), this, SLOT(onPushButtonRunScript()));
   connect(m_pUi->pushButtonStopScript, SIGNAL(clicked()), this, SLOT(onPushButtonStopScript()));

   // speed factor
   EcInt32 currentSpeedFactor = m_pUi->progressBarSpeedFactor->value();
   m_pControlPluginExec->setUserSpeedFactor(currentSpeedFactor / 100.0);

   // timer
   connect(&m_GuiUpdateTimer, SIGNAL(timeout()), this, SLOT(update()));
   m_GuiUpdateTimer.start(50);

   m_QMsgBox.setText("EMERGENCY STOPPED!");
}

//------------------------------------------------------------------------------
EcBoolean EcMkControlWidget::initState()
{
   // get manipulator
   m_PluginGui.getParam<Ec::Manipulator>(EcRtos::MANIP_INDEX, m_MkManipulator);
   m_GravitationalTorqueTool.setManipulator(&m_MkManipulator);
   m_GravitationalTorqueTool.setGravityVector(EcVector(0.0, 0.0, -9.8));

   return EcTrue;
}

//------------------------------------------------------------------------------
void EcMkControlWidget::update()
{
   m_PluginGui.setParam<Ec::RenderVirtual>(EcRtos::MANIP_INDEX, EcTrue);

   /// get states from exec plugin
   {
      // get state machine state
      m_StateMachineState = m_pControlPluginExec->getStateMachineState();
      m_BrakesStatus = m_pControlPluginExec->getAxisState();
      m_AxisFaults = m_pControlPluginExec->getAxisFault();

      // get joint torques
      m_JointTorquesInst = m_pControlPluginExec->getJointTorques();
   }

   // display state machine state
   EcRtos::State stateMachineState = static_cast<EcRtos::State>(m_StateMachineState);
   switch (stateMachineState)
   {
   case EcRtos::State::START:
      m_pUi->labelMkState->setText("Standby");
      break;
   case EcRtos::State::DISABLED:
      m_pUi->labelMkState->setText("Disabled");
      break;
   case EcRtos::State::ENABLING:
      m_pUi->labelMkState->setText("Enabling");
      break;
   case EcRtos::State::ENABLED:
      m_pUi->labelMkState->setText("Enabled");
      break;
   case EcRtos::State::DISABLING:
      m_pUi->labelMkState->setText("Disabling");
      break;
   case EcRtos::State::FAULT:
      m_pUi->labelMkState->setText("Fault");
      break;
   case EcRtos::State::STOPPING:
      m_pUi->labelMkState->setText("STOPPING");
      break;
   case EcRtos::State::STOPPED:
      m_pUi->labelMkState->setText("STOPPED");
      break;
   case EcRtos::State::SIMULATION:
      m_pUi->labelMkState->setText("Simulation");
      break;
   case EcRtos::State::E_STOPPED:
      m_pUi->labelMkState->setText("EMERGENCY STOPPED");
      m_RequestedOperation = EcRtos::Operation::STANDBY;
      m_pControlPluginExec->setOperation(m_RequestedOperation);
      m_QMsgBox.show();
      disconnectRtos();
      break;
   default:
      break;
   }

   // display operation
   EcRtos::Operation operation = static_cast<EcRtos::Operation>(m_RequestedOperation);
   switch (operation)
   {
   case EcRtos::Operation::STANDBY:
      m_pUi->labelRequestedOperation->setText("Standby");
      break;
   case EcRtos::Operation::INITIALIZE:
      m_pUi->labelRequestedOperation->setText("Initialize");
      break;
   case EcRtos::Operation::RESET:
      m_pUi->labelRequestedOperation->setText("Reset");
      break;
   case EcRtos::Operation::ENABLE:
      m_pUi->labelRequestedOperation->setText("Enable");
      break;
   case EcRtos::Operation::DISABLE:
      m_pUi->labelRequestedOperation->setText("Disable");
      break;
   case EcRtos::Operation::STOP:
      m_pUi->labelRequestedOperation->setText("STOP");
      break;
   case EcRtos::Operation::SIMULATE:
      m_pUi->labelRequestedOperation->setText("Simulate");
      break;
   case EcRtos::Operation::STOP_SIMULATION:
      m_pUi->labelRequestedOperation->setText("Stop Simulation");
      break;
   default:
      break;
   }

   // watchdog
   m_pUi->labelWatchdog->setText(QString::number(m_pControlPluginExec->watchdog()));

   // CPU
   m_pUi->labelCpuLoad->setText(QString::number(m_pControlPluginExec->getCpuLoad()));

   // digital inputs/outputs
   m_pUi->labelDigitalInputs->setText("0x" + QString::number(m_pControlPluginExec->getDigitalInputs(), 16));
   m_pUi->labelDigitalOutputs->setText("0x" + QString::number(m_pControlPluginExec->getDigitalOutputs(), 16));

   // display joint state
   for (EcSizeT ii = 0; ii < EcRtos::MK_NUM_AXES; ++ii)
   {
      EcRtos::BrakeStatus brakeStatus = static_cast<EcRtos::BrakeStatus>(m_BrakesStatus[ii]);
      switch (brakeStatus)
      {
      case EcRtos::BrakeStatus::HOLD_ZERO:
         m_BrakeStatusLabels[ii]->setText("Hold Zero");
         break;
      case EcRtos::BrakeStatus::HOLDING:
         m_BrakeStatusLabels[ii]->setText("Holding");
         break;
      case EcRtos::BrakeStatus::RELEASING:
         m_BrakeStatusLabels[ii]->setText("Enabling");
         break;
      case EcRtos::BrakeStatus::LOCKING:
         m_BrakeStatusLabels[ii]->setText("Locking");
         break;
      case EcRtos::BrakeStatus::RELEASED:
         m_BrakeStatusLabels[ii]->setText("Enabled");
         break;
      case EcRtos::BrakeStatus::LOCKED:
         m_BrakeStatusLabels[ii]->setText("Disabled");
         break;
      default:
         m_BrakeStatusLabels[ii]->setText("Unknown");
         break;
      }

      m_AxisFaultLabels[ii]->setText(QString::number(m_AxisFaults[ii], 16));
   }

   // torques
   if (m_PluginGui.param<Ec::SimRunState, Ec::SimulationRunState>() == Ec::SimulationRunning)
   {
      // calculate expected
      m_PluginGui.getParam<Ec::State>(EcRtos::MANIP_INDEX, m_PositionState);
      m_GravitationalTorqueTool.setPositionState(m_PositionState);
      m_GravitationalTorques = m_GravitationalTorqueTool.gravitationalTorques();

      for (EcSizeT ii = 0; ii < EcRtos::MK_NUM_AXES; ++ii)
      {
         m_JointTorqueInstLabels[ii]->setText(QString("%1").arg(m_JointTorquesInst[ii], 0, 'f', 6, '0'));
         m_JointTorqueAvgLabels[ii]->setText(QString("%1").arg(m_JointTorquesAvg[ii], 0, 'f', 6, '0'));
         m_JointTorqueExpLabels[ii]->setText(QString("%1").arg(m_GravitationalTorques[ii][0], 0, 'f', 6, '0'));

         m_JointTorquesAvg[ii] = m_RunningAverageFilters[ii].updateOutput(m_JointTorquesInst[ii]);
      }
   }

   // display EE set
   m_PluginGui.getParam<Ec::EndEffectorSet>(EcRtos::MANIP_INDEX, m_EeSetIndex);
   if (m_EeSetIndex == EcVelocityController::JOINT_CONTROL_INDEX)
   {
      m_pUi->labelCurrentEeSet->setText("Joint Control");
   }
   else
   {
      m_pUi->labelCurrentEeSet->setText(QString::number(m_EeSetIndex));
   }

   // update ee placement
   updateEePlacement();

   // motion script status
   if (m_pUi->tabWidget->currentIndex() == 2)
   {
      // in tab script
      EcBoolean runningMotionScript;
      EcInt32 motionScriptStatus;
      m_pControlPluginExec->getMotionScriptStatus(runningMotionScript, motionScriptStatus);
      switch (static_cast<EcMotionScriptObject::Status>(motionScriptStatus))
      {
      case EcMotionScriptObject::NOT_STARTED:
         m_pUi->textBrowserOutput->setText("Not started");
         break;
      case EcMotionScriptObject::IN_PROGRESS:
         m_pUi->textBrowserOutput->setText("In progress...");
         break;
      case EcMotionScriptObject::SUCCEEDED:
         m_pUi->textBrowserOutput->setText("Succeeded");
         break;
      case EcMotionScriptObject::FAILED:
         m_pUi->textBrowserOutput->setText("Failed");
         break;
      default:
         break;
      }
   }
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonInitClicked()
{
   if (!m_pControlPluginExec->initConnection())
   {
      m_pUi->labelMkState->setText("Init Error!");
      return;
   }

   std::cout << "EcMkControlWidget::onPushButtonInitClicked(): Initializing..." << std::endl;
   m_pUi->labelMkState->setText("Initializing");
   m_RequestedOperation = EcRtos::Operation::INITIALIZE;
   m_pControlPluginExec->setOperation(m_RequestedOperation);

   // Render virtual system
   m_PluginGui.setParam<Ec::DisplayMask>(EcRenderTypes::VirtualManipulator, EcTrue);

   // start simulation
   m_PluginGui.setParam<Ec::SimRunState>(Ec::SimulationRunning);

   // wait for state change
   EcU32 trials = 0;
   while (m_pControlPluginExec->getStateMachineState() == static_cast<EcInt32>(EcRtos::State::START))
   {
      if (++trials > 10)
      {
         disconnectRtos();
         return;
      }
      EcSLEEPMS(100);
   }

   // disable init
   m_pUi->pushButtonInit->setEnabled(false);

   // enable buttons
   m_pUi->pushButtonReset->setEnabled(true);
   m_pUi->pushButtonEnable->setEnabled(true);
   m_pUi->pushButtonDisable->setEnabled(true);
   m_pUi->pushButtonStop->setEnabled(true);
   m_pUi->pushButtonSimulate->setEnabled(true);
   m_pUi->pushButtonStopSimulation->setEnabled(true);
   m_pUi->pushButtonHome->setEnabled(true);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonResetClicked()
{
   m_RequestedOperation = EcRtos::Operation::RESET;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonEnableClicked()
{
   m_pManipulationDirector->stopDirector();
   m_RequestedOperation = EcRtos::Operation::ENABLE;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonDisableClicked()
{
   m_RequestedOperation = EcRtos::Operation::DISABLE;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonSimulateClicked()
{
   m_RequestedOperation = EcRtos::Operation::SIMULATE;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonStopSimulationClicked()
{
   m_RequestedOperation = EcRtos::Operation::STOP_SIMULATION;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonStopClicked()
{
   m_pManipulationDirector->stopDirector();
   m_RequestedOperation = EcRtos::Operation::STOP;
   m_pControlPluginExec->setOperation(m_RequestedOperation);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonHomePressed()
{
   m_pControlPluginExec->setHomingOn(EcTrue);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonHomeReleased()
{
   m_pControlPluginExec->setHomingOn(EcFalse);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonDecreaseSpeedFactorClicked()
{
   EcInt32 currentSpeedFactor = m_pUi->progressBarSpeedFactor->value() - 5;
   m_pUi->progressBarSpeedFactor->setValue(currentSpeedFactor);
   currentSpeedFactor = m_pUi->progressBarSpeedFactor->value();
   m_pControlPluginExec->setUserSpeedFactor(currentSpeedFactor / 100.0);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonIncreaseSpeedFactorClicked()
{
   EcInt32 currentSpeedFactor = m_pUi->progressBarSpeedFactor->value() + 5;
   m_pUi->progressBarSpeedFactor->setValue(currentSpeedFactor);
   currentSpeedFactor = m_pUi->progressBarSpeedFactor->value();
   m_pControlPluginExec->setUserSpeedFactor(currentSpeedFactor / 100.0);
}

//------------------------------------------------------------------------------
EcReal
EcMkControlWidget::getStepSize()
{
   EcReal retVal = 1.0;
   switch (m_pUi->horizontalSliderStep->value())
   {
   case 1:
      retVal = 0.01;
      break;
   case 2:
      retVal = 0.1;
      break;
   case 3:
      retVal = 1.0;
      break;
   case 4:
      retVal = 10.0;
      break;
   case 5:
      retVal = 20.0;
      break;
   }

   return retVal;
}

//------------------------------------------------------------------------------
void EcMkControlWidget::onPushButtonJointDown0Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint0->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint0->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointDown1Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint1->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint1->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointDown2Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint2->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint2->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointDown3Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint3->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint3->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointDown4Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint4->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint4->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointDown5Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint5->value();
   jointAngle -= 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint5->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp0Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint0->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint0->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp1Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint1->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint1->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp2Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint2->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint2->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp3Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint3->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint3->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp4Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint4->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint4->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonJointUp5Clicked()
{
   EcReal stepSize = getStepSize();
   EcReal jointAngle = m_pUi->doubleSpinBoxJoint5->value();
   jointAngle += 1.0 * stepSize;
   m_pUi->doubleSpinBoxJoint5->setValue(jointAngle);
   setDesiredJoint();
}

void EcMkControlWidget::onPushButtonEeDownXClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::xVector(-stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeDownYClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::yVector(-stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeDownZClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::zVector(-stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeDownRollClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::xRotation(-stepSize * EcDEG2RAD));
}

void EcMkControlWidget::onPushButtonEeDownPitchClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::yRotation(-stepSize * EcDEG2RAD));
}

void EcMkControlWidget::onPushButtonEeDownYawClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::zRotation(-stepSize * EcDEG2RAD));
}

void EcMkControlWidget::onPushButtonEeUpXClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::xVector(stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeUpYClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::yVector(stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeUpZClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredTranslation(EcVector::zVector(stepSize * EcMILLI));
}

void EcMkControlWidget::onPushButtonEeUpRollClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::xRotation(stepSize * EcDEG2RAD));
}

void EcMkControlWidget::onPushButtonEeUpPitchClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::yRotation(stepSize * EcDEG2RAD));
}

void EcMkControlWidget::onPushButtonEeUpYawClicked()
{
   EcReal stepSize = getStepSize();
   setDesiredOrientation(EcOrientation::zRotation(stepSize * EcDEG2RAD));
}

//------------------------------------------------------------------------------
void
EcMkControlWidget::onPushButtonRunScript
   (
   )
{
   EcString script = m_pUi->plainTextEditScript->toPlainText().toStdString();
   EcString errMsg;
   EcMotionScriptSharedPointer pMotionScript(EcMotionScriptParser::getInstance().parseScript(script, &errMsg));
   if (!pMotionScript)
   {
      m_pUi->textBrowserOutput->setText(QString::fromStdString(errMsg));
      return;
   }
   m_pControlPluginExec->sendMotionScript(script);
}

//------------------------------------------------------------------------------
void
EcMkControlWidget::onPushButtonStopScript
   (
   )
{
   EcEndEffectorMotionStop stopMotion;
   stopMotion.setManipulatorIndex(EcRtos::MANIP_INDEX);
   stopMotion.stop();
   m_pControlPluginExec->sendMotionScript(stopMotion.write());
}

//------------------------------------------------------------------------------
void EcMkControlWidget::updateEePlacement()
{
   // update actual joint angles
   m_PluginGui.getParam<Ec::JointAngle | Ec::VirtualSystem>(EcRtos::MANIP_INDEX, m_ActualJointValues);
   const EcEndEffectorPlacement& actualJointEe = m_ManipulatorEePlacement.offsetTransformations()[0];
   for (EcSizeT ii = 0; ii < EcRtos::MK_NUM_AXES; ++ii)
   {
      m_ActualJointLabels[ii]->setText(QString("%1").arg(m_ActualJointValues[ii] * EcRAD2DEG, 0, 'f', 4, '0'));
   }

   // update command joint angles
   m_PluginGui.getParam<Ec::ActualEndEffector>(EcRtos::MANIP_INDEX, EcVelocityController::JOINT_CONTROL_INDEX, m_ManipulatorEePlacement);
   const EcEndEffectorPlacement& jointEe = m_ManipulatorEePlacement.offsetTransformations()[0];
   m_pUi->doubleSpinBoxJoint0->setValue(jointEe.data()[0] * EcRAD2DEG);
   m_pUi->doubleSpinBoxJoint1->setValue(jointEe.data()[1] * EcRAD2DEG);
   m_pUi->doubleSpinBoxJoint2->setValue(jointEe.data()[2] * EcRAD2DEG);
   m_pUi->doubleSpinBoxJoint3->setValue(jointEe.data()[3] * EcRAD2DEG);
   m_pUi->doubleSpinBoxJoint4->setValue(jointEe.data()[4] * EcRAD2DEG);
   m_pUi->doubleSpinBoxJoint5->setValue(jointEe.data()[5] * EcRAD2DEG);

   // update actual frame
   m_PluginGui.getParam<Ec::ActualEndEffector | Ec::VirtualSystem>(EcRtos::MANIP_INDEX, EcRtos::FRAME_EE_INDEX, m_ManipulatorEePlacement);
   const EcEndEffectorPlacement& actualFrameEe = m_ManipulatorEePlacement.offsetTransformations()[0];
   m_pUi->labelActualEeX->setText(QString("%1").arg(actualFrameEe.coordSysXForm().translation().x() * 1000, 0, 'f', 4, '0'));
   m_pUi->labelActualEeY->setText(QString("%1").arg(actualFrameEe.coordSysXForm().translation().y() * 1000, 0, 'f', 4, '0'));
   m_pUi->labelActualEeZ->setText(QString("%1").arg(actualFrameEe.coordSysXForm().translation().z() * 1000, 0, 'f', 4, '0'));
   EcReal roll, pitch, yaw;
   actualFrameEe.coordSysXForm().orientation().get123Euler(roll, pitch, yaw);
   m_pUi->labelActualEeRoll->setText(QString("%1").arg(roll * EcRAD2DEG, 0, 'f', 4, '0'));
   m_pUi->labelActualEePitch->setText(QString("%1").arg(pitch * EcRAD2DEG, 0, 'f', 4, '0'));
   m_pUi->labelActualEeYaw->setText(QString("%1").arg(yaw * EcRAD2DEG, 0, 'f', 4, '0'));

   // update command frame
   m_PluginGui.getParam<Ec::ActualEndEffector>(EcRtos::MANIP_INDEX, EcRtos::FRAME_EE_INDEX, m_ManipulatorEePlacement);
   const EcEndEffectorPlacement& frameEe = m_ManipulatorEePlacement.offsetTransformations()[0];
   m_pUi->doubleSpinBoxX->setValue(frameEe.coordSysXForm().translation().x() * 1000);
   m_pUi->doubleSpinBoxY->setValue(frameEe.coordSysXForm().translation().y() * 1000);
   m_pUi->doubleSpinBoxZ->setValue(frameEe.coordSysXForm().translation().z() * 1000);
   frameEe.coordSysXForm().orientation().get123Euler(roll, pitch, yaw);
   m_pUi->doubleSpinBoxRoll->setValue(roll * EcRAD2DEG);
   m_pUi->doubleSpinBoxPitch->setValue(pitch * EcRAD2DEG);
   m_pUi->doubleSpinBoxYaw->setValue(yaw * EcRAD2DEG);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::setDesiredJoint()
{
   m_PluginGui.setParam<Ec::EndEffectorSet>(EcRtos::MANIP_INDEX, static_cast<EcU32>(EcVelocityController::JOINT_CONTROL_INDEX));
   m_DesiredEePlacement.data().resize(EcRtos::MK_NUM_AXES);
   m_DesiredEePlacement.data()[0] = m_pUi->doubleSpinBoxJoint0->value() * EcDEG2RAD;
   m_DesiredEePlacement.data()[1] = m_pUi->doubleSpinBoxJoint1->value() * EcDEG2RAD;
   m_DesiredEePlacement.data()[2] = m_pUi->doubleSpinBoxJoint2->value() * EcDEG2RAD;
   m_DesiredEePlacement.data()[3] = m_pUi->doubleSpinBoxJoint3->value() * EcDEG2RAD;
   m_DesiredEePlacement.data()[4] = m_pUi->doubleSpinBoxJoint4->value() * EcDEG2RAD;
   m_DesiredEePlacement.data()[5] = m_pUi->doubleSpinBoxJoint5->value() * EcDEG2RAD;
   m_PluginGui.setParam<Ec::DesiredEndEffector>(EcRtos::MANIP_INDEX, 0, m_DesiredEePlacement);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::setDesiredTranslation(const EcVector& vec)
{
   // get the pose in system frame
   m_PluginGui.getParam<Ec::ActualEndEffector>(EcRtos::MANIP_INDEX, EcRtos::FRAME_EE_INDEX, m_ManipulatorEePlacement);
   EcCoordinateSystemTransformation xform = m_ManipulatorEePlacement.offsetTransformations()[0].coordSysXForm();

   // set command
   if (m_pUi->checkBoxJoggingEeLocal->isChecked())
   {
      // local frame
      xform.setTranslation(xform * vec);
   }
   else
   {
      // global frame
      xform = EcCoordinateSystemTransformation(vec) * xform;
   }

   m_PluginGui.setParam<Ec::EndEffectorSet>(EcRtos::MANIP_INDEX, static_cast<EcU32>(EcRtos::FRAME_EE_INDEX));
   m_DesiredEePlacement.coordSysXForm() = xform;
   m_PluginGui.setParam<Ec::DesiredEndEffector>(EcRtos::MANIP_INDEX, 0, m_DesiredEePlacement);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::setDesiredOrientation(const EcOrientation& ori)
{
   // get the pose in system frame
   m_PluginGui.getParam<Ec::ActualEndEffector>(EcRtos::MANIP_INDEX, EcRtos::FRAME_EE_INDEX, m_ManipulatorEePlacement);
   EcCoordinateSystemTransformation xform = m_ManipulatorEePlacement.offsetTransformations()[0].coordSysXForm();

   // set command
   if (m_pUi->checkBoxJoggingEeLocal->isChecked())
   {
      // local frame
      xform.setOrientation(xform.orientation() * ori);
   }
   else
   {
      // global frame
      xform.setOrientation(ori * xform.orientation());
   }

   m_PluginGui.setParam<Ec::EndEffectorSet>(EcRtos::MANIP_INDEX, static_cast<EcU32>(EcRtos::FRAME_EE_INDEX));
   m_DesiredEePlacement.coordSysXForm() = xform;
   m_PluginGui.setParam<Ec::DesiredEndEffector>(EcRtos::MANIP_INDEX, 0, m_DesiredEePlacement);
}

//------------------------------------------------------------------------------
void EcMkControlWidget::disconnectRtos()
{
   m_pControlPluginExec->disconnectRtos();

   // enable/disable buttons
   m_pUi->pushButtonInit->setEnabled(true);
   m_pUi->pushButtonReset->setEnabled(false);
   m_pUi->pushButtonEnable->setEnabled(false);
   m_pUi->pushButtonDisable->setEnabled(false);
   m_pUi->pushButtonStop->setEnabled(false);
   m_pUi->pushButtonSimulate->setEnabled(false);
   m_pUi->pushButtonStopSimulation->setEnabled(false);
   m_pUi->pushButtonHome->setEnabled(false);
}
