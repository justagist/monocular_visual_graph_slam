/********************************************************************************
** Form generated from reading UI file 'base_main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASE_MAIN_WINDOW_H
#define UI_BASE_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "g2o_qglviewer.h"

QT_BEGIN_NAMESPACE

class Ui_BaseMainWindow
{
public:
    QAction *actionLoad;
    QAction *actionSave;
    QAction *actionQuit;
    QAction *actionWhite_Background;
    QAction *actionDefault_Background;
    QAction *actionDump_Images;
    QAction *actionProperties;
    QAction *actionSave_Screenshot;
    QAction *actionSave_Viewer_State;
    QAction *actionLoad_Viewer_State;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *cbDrawAxis;
    QLabel *label;
    QSpinBox *spIterations;
    QCheckBox *cbRobustKernel;
    QCheckBox *cbOnlyLoop;
    QComboBox *coRobustKernel;
    QLabel *label_4;
    QLineEdit *leKernelWidth;
    QLabel *label_2;
    QComboBox *coOptimizer;
    QPushButton *btnOptimizerParamaters;
    QSpacerItem *verticalSpacer_2;
    QComboBox *cbxIniitialGuessMethod;
    QPushButton *btnInitialGuess;
    QPushButton *btnReload;
    QPushButton *btnSetZero;
    QPushButton *btnOptimize;
    QPushButton *btnForceStop;
    QPushButton *btnQuit;
    QSpacerItem *verticalSpacer;
    QSplitter *splitter;
    g2o::G2oQGLViewer *viewer;
    QPlainTextEdit *plainTextEdit;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuView;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *BaseMainWindow)
    {
        if (BaseMainWindow->objectName().isEmpty())
            BaseMainWindow->setObjectName(QStringLiteral("BaseMainWindow"));
        BaseMainWindow->resize(800, 600);
        actionLoad = new QAction(BaseMainWindow);
        actionLoad->setObjectName(QStringLiteral("actionLoad"));
        actionSave = new QAction(BaseMainWindow);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionQuit = new QAction(BaseMainWindow);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        actionWhite_Background = new QAction(BaseMainWindow);
        actionWhite_Background->setObjectName(QStringLiteral("actionWhite_Background"));
        actionDefault_Background = new QAction(BaseMainWindow);
        actionDefault_Background->setObjectName(QStringLiteral("actionDefault_Background"));
        actionDump_Images = new QAction(BaseMainWindow);
        actionDump_Images->setObjectName(QStringLiteral("actionDump_Images"));
        actionDump_Images->setCheckable(true);
        actionProperties = new QAction(BaseMainWindow);
        actionProperties->setObjectName(QStringLiteral("actionProperties"));
        actionSave_Screenshot = new QAction(BaseMainWindow);
        actionSave_Screenshot->setObjectName(QStringLiteral("actionSave_Screenshot"));
        actionSave_Viewer_State = new QAction(BaseMainWindow);
        actionSave_Viewer_State->setObjectName(QStringLiteral("actionSave_Viewer_State"));
        actionLoad_Viewer_State = new QAction(BaseMainWindow);
        actionLoad_Viewer_State->setObjectName(QStringLiteral("actionLoad_Viewer_State"));
        centralwidget = new QWidget(BaseMainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QStringLiteral("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(150, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_2 = new QVBoxLayout(frame);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        cbDrawAxis = new QCheckBox(frame);
        cbDrawAxis->setObjectName(QStringLiteral("cbDrawAxis"));
        cbDrawAxis->setChecked(true);

        verticalLayout_2->addWidget(cbDrawAxis);

        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_2->addWidget(label);

        spIterations = new QSpinBox(frame);
        spIterations->setObjectName(QStringLiteral("spIterations"));
        spIterations->setMinimum(1);
        spIterations->setMaximum(10000);
        spIterations->setValue(10);

        verticalLayout_2->addWidget(spIterations);

        cbRobustKernel = new QCheckBox(frame);
        cbRobustKernel->setObjectName(QStringLiteral("cbRobustKernel"));

        verticalLayout_2->addWidget(cbRobustKernel);

        cbOnlyLoop = new QCheckBox(frame);
        cbOnlyLoop->setObjectName(QStringLiteral("cbOnlyLoop"));
        cbOnlyLoop->setEnabled(false);

        verticalLayout_2->addWidget(cbOnlyLoop);

        coRobustKernel = new QComboBox(frame);
        coRobustKernel->setObjectName(QStringLiteral("coRobustKernel"));
        coRobustKernel->setEnabled(false);

        verticalLayout_2->addWidget(coRobustKernel);

        label_4 = new QLabel(frame);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setEnabled(false);

        verticalLayout_2->addWidget(label_4);

        leKernelWidth = new QLineEdit(frame);
        leKernelWidth->setObjectName(QStringLiteral("leKernelWidth"));
        leKernelWidth->setEnabled(false);

        verticalLayout_2->addWidget(leKernelWidth);

        label_2 = new QLabel(frame);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_2->addWidget(label_2);

        coOptimizer = new QComboBox(frame);
        coOptimizer->setObjectName(QStringLiteral("coOptimizer"));

        verticalLayout_2->addWidget(coOptimizer);

        btnOptimizerParamaters = new QPushButton(frame);
        btnOptimizerParamaters->setObjectName(QStringLiteral("btnOptimizerParamaters"));

        verticalLayout_2->addWidget(btnOptimizerParamaters);

        verticalSpacer_2 = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Minimum);

        verticalLayout_2->addItem(verticalSpacer_2);

        cbxIniitialGuessMethod = new QComboBox(frame);
        cbxIniitialGuessMethod->setObjectName(QStringLiteral("cbxIniitialGuessMethod"));

        verticalLayout_2->addWidget(cbxIniitialGuessMethod);

        btnInitialGuess = new QPushButton(frame);
        btnInitialGuess->setObjectName(QStringLiteral("btnInitialGuess"));

        verticalLayout_2->addWidget(btnInitialGuess);

        btnReload = new QPushButton(frame);
        btnReload->setObjectName(QStringLiteral("btnReload"));

        verticalLayout_2->addWidget(btnReload);

        btnSetZero = new QPushButton(frame);
        btnSetZero->setObjectName(QStringLiteral("btnSetZero"));

        verticalLayout_2->addWidget(btnSetZero);

        btnOptimize = new QPushButton(frame);
        btnOptimize->setObjectName(QStringLiteral("btnOptimize"));

        verticalLayout_2->addWidget(btnOptimize);

        btnForceStop = new QPushButton(frame);
        btnForceStop->setObjectName(QStringLiteral("btnForceStop"));

        verticalLayout_2->addWidget(btnForceStop);

        btnQuit = new QPushButton(frame);
        btnQuit->setObjectName(QStringLiteral("btnQuit"));

        verticalLayout_2->addWidget(btnQuit);

        verticalSpacer = new QSpacerItem(20, 364, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout->addWidget(frame);

        splitter = new QSplitter(centralwidget);
        splitter->setObjectName(QStringLiteral("splitter"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(splitter->sizePolicy().hasHeightForWidth());
        splitter->setSizePolicy(sizePolicy1);
        splitter->setOrientation(Qt::Vertical);
        viewer = new g2o::G2oQGLViewer(splitter);
        viewer->setObjectName(QStringLiteral("viewer"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(100);
        sizePolicy2.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy2);
        splitter->addWidget(viewer);
        plainTextEdit = new QPlainTextEdit(splitter);
        plainTextEdit->setObjectName(QStringLiteral("plainTextEdit"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(1);
        sizePolicy3.setHeightForWidth(plainTextEdit->sizePolicy().hasHeightForWidth());
        plainTextEdit->setSizePolicy(sizePolicy3);
        plainTextEdit->setReadOnly(true);
        splitter->addWidget(plainTextEdit);

        horizontalLayout->addWidget(splitter);

        BaseMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(BaseMainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QStringLiteral("menuView"));
        BaseMainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(BaseMainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        BaseMainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menuFile->addAction(actionLoad);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionSave_Screenshot);
        menuFile->addSeparator();
        menuFile->addAction(actionLoad_Viewer_State);
        menuFile->addAction(actionSave_Viewer_State);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuView->addAction(actionWhite_Background);
        menuView->addAction(actionDefault_Background);
        menuView->addSeparator();
        menuView->addAction(actionDump_Images);
        menuView->addSeparator();
        menuView->addAction(actionProperties);

        retranslateUi(BaseMainWindow);
        QObject::connect(cbRobustKernel, SIGNAL(toggled(bool)), label_4, SLOT(setEnabled(bool)));
        QObject::connect(cbRobustKernel, SIGNAL(toggled(bool)), leKernelWidth, SLOT(setEnabled(bool)));
        QObject::connect(btnQuit, SIGNAL(clicked()), BaseMainWindow, SLOT(close()));
        QObject::connect(cbRobustKernel, SIGNAL(toggled(bool)), coRobustKernel, SLOT(setEnabled(bool)));
        QObject::connect(cbRobustKernel, SIGNAL(toggled(bool)), cbOnlyLoop, SLOT(setEnabled(bool)));

        QMetaObject::connectSlotsByName(BaseMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *BaseMainWindow)
    {
        BaseMainWindow->setWindowTitle(QApplication::translate("BaseMainWindow", "g2o Viewer", 0));
        actionLoad->setText(QApplication::translate("BaseMainWindow", "Load", 0));
        actionSave->setText(QApplication::translate("BaseMainWindow", "Save", 0));
        actionQuit->setText(QApplication::translate("BaseMainWindow", "Quit", 0));
        actionWhite_Background->setText(QApplication::translate("BaseMainWindow", "White Background", 0));
        actionDefault_Background->setText(QApplication::translate("BaseMainWindow", "Default Background", 0));
        actionDump_Images->setText(QApplication::translate("BaseMainWindow", "Dump Images", 0));
        actionProperties->setText(QApplication::translate("BaseMainWindow", "Draw Options", 0));
        actionSave_Screenshot->setText(QApplication::translate("BaseMainWindow", "Save Screenshot", 0));
        actionSave_Viewer_State->setText(QApplication::translate("BaseMainWindow", "Save Viewer State", 0));
        actionLoad_Viewer_State->setText(QApplication::translate("BaseMainWindow", "Load Viewer State", 0));
        cbDrawAxis->setText(QApplication::translate("BaseMainWindow", "Draw Axis", 0));
        label->setText(QApplication::translate("BaseMainWindow", "# Iterations", 0));
        cbRobustKernel->setText(QApplication::translate("BaseMainWindow", "Robust Kernel", 0));
#ifndef QT_NO_TOOLTIP
        cbOnlyLoop->setToolTip(QApplication::translate("BaseMainWindow", "Only apply the robust kernel for loop closures", 0));
#endif // QT_NO_TOOLTIP
        cbOnlyLoop->setText(QApplication::translate("BaseMainWindow", "Non Sequential", 0));
        label_4->setText(QApplication::translate("BaseMainWindow", "Kernel Width", 0));
        leKernelWidth->setText(QApplication::translate("BaseMainWindow", "1.0", 0));
        label_2->setText(QApplication::translate("BaseMainWindow", "Optimizer", 0));
#ifndef QT_NO_TOOLTIP
        btnOptimizerParamaters->setToolTip(QApplication::translate("BaseMainWindow", "Adjust the parameters of the optimizer", 0));
#endif // QT_NO_TOOLTIP
        btnOptimizerParamaters->setText(QApplication::translate("BaseMainWindow", "Parameters", 0));
        cbxIniitialGuessMethod->clear();
        cbxIniitialGuessMethod->insertItems(0, QStringList()
         << QApplication::translate("BaseMainWindow", "Spanning Tree", 0)
         << QApplication::translate("BaseMainWindow", "Odometry", 0)
        );
#ifndef QT_NO_TOOLTIP
        cbxIniitialGuessMethod->setToolTip(QApplication::translate("BaseMainWindow", "Which method is applied to compute the initial guess", 0));
#endif // QT_NO_TOOLTIP
        btnInitialGuess->setText(QApplication::translate("BaseMainWindow", "Initial Guess", 0));
        btnReload->setText(QApplication::translate("BaseMainWindow", "Reload", 0));
        btnSetZero->setText(QApplication::translate("BaseMainWindow", "SetZero", 0));
        btnOptimize->setText(QApplication::translate("BaseMainWindow", "Optimize", 0));
        btnForceStop->setText(QApplication::translate("BaseMainWindow", "Stop", 0));
        btnQuit->setText(QApplication::translate("BaseMainWindow", "Quit", 0));
        menuFile->setTitle(QApplication::translate("BaseMainWindow", "File", 0));
        menuView->setTitle(QApplication::translate("BaseMainWindow", "View", 0));
    } // retranslateUi

};

namespace Ui {
    class BaseMainWindow: public Ui_BaseMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_MAIN_WINDOW_H
