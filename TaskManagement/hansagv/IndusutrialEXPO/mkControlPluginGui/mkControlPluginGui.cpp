//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file mkControlPluginGui.cpp
//
//------------------------------------------------------------------------------
#include "mkControlPluginGui.h"
#include "ecMkControlWidget.h"
#include <manipulationDirectorPlugin/manipulationDirectorPlugin.h>
#include <mkControlPluginExec/mkControlPluginExec.h>

#include <QtWidgets/QAction>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QMenu>

EC_PLUGIN_STUB_DEFAULT(mkControlPluginGui);

//------------------------------------------------------------------------------
mkControlPluginGui::mkControlPluginGui
   (
   ) :
Ec::PluginGUI(),
m_pDockWidget(EcNULL),
m_pControlWidget(EcNULL)
{
   // needed for the slots
   setObjectName("mkControlPluginGui");

   m_RequiredPlugins.push_back("mkControlPluginExec");
   m_RequiredPlugins.push_back("manipulationDirectorPlugin");
}

//------------------------------------------------------------------------------
EcBoolean mkControlPluginGui::init
   (
   )
{
   m_pControlPluginExec = EcPLUGIN_FIND_AND_CAST(mkControlPluginExec);
   if (!m_pControlPluginExec)
   {
      return EcFalse;
   }

   m_pManipulationDirector = EcPLUGIN_FIND_AND_CAST(manipulationDirectorPlugin);
   if (!m_pManipulationDirector)
   {
      return EcFalse;
   }

   // add menu and initialize dock widget
   QMenu* menuEdit = getOrAddMenu("&Edit", "Plugins");
   if (!menuEdit)
   {
      return EcFalse;
   }
   QAction* action = new QAction(tr("&MK Control"), this);
   connect(action, SIGNAL(triggered()), SLOT(showControlWidget()));
   menuEdit->addAction(action);
   m_pDockWidget = new QDockWidget(tr("MK Control"));
   m_pDockWidget->setObjectName(QString::fromUtf8("mkControlWidget"));
   m_pDockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
   addDockWidget(Qt::RightDockWidgetArea, m_pDockWidget);

   // initialize widget
   m_pControlWidget = new EcMkControlWidget(*this, m_pControlPluginExec, m_pManipulationDirector);
   m_pDockWidget->setWidget(m_pControlWidget);
   m_pDockWidget->show();
   m_pDockWidget->raise();

   return EcTrue;
}

//------------------------------------------------------------------------------
EcBoolean mkControlPluginGui::initState
   (
   )
{
   return m_pControlWidget->initState();
}

//------------------------------------------------------------------------------
void
mkControlPluginGui::showControlWidget
   (
   )
{
   m_pDockWidget->show();
   m_pDockWidget->raise();
}
