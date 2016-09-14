#ifndef mkControlPluginGui_H_
#define mkControlPluginGui_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file mkControlPluginGui.h
/// @class mkControlPluginGui
/// @brief MK robot control plugin GUI
/// @details This plugin controls a MK robot
//
//------------------------------------------------------------------------------
#include <viewerCore/ecPluginGUI.h>

// forward declaration
class EcMkControlWidget;
class mkControlPluginExec;
class manipulationDirectorPlugin;

/// MK robot control plugin GUI
class mkControlPluginGui : public Ec::PluginGUI
{
   Q_OBJECT

public:
   /// constructor
   mkControlPluginGui
      (
      );

   /// @copydoc Ec::PluginGUI::init
   virtual EcBoolean init
      (
      );

   /// @copydoc Ec::PluginGUI::initState
   virtual EcBoolean initState
      (
      );

protected Q_SLOTS:
   /// show control widget
   void showControlWidget
      (
      );

protected:
   QDockWidget*                           m_pDockWidget;                ///< the dock widget containing the widget
   EcMkControlWidget*                     m_pControlWidget;             ///< the widget
   mkControlPluginExec*                   m_pControlPluginExec;         ///< the control exec plugin
   manipulationDirectorPlugin*            m_pManipulationDirector;      ///< the manipulation director plugin
};

#endif // mkControlPluginGui_H_
