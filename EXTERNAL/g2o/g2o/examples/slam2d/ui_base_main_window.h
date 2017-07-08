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
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "slam2d_viewer.h"

QT_BEGIN_NAMESPACE

class Ui_BaseMainWindow
{
public:
    QAction *actionLoad;
    QAction *actionSave;
    QAction *actionQuit;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QSpinBox *spIterations;
    QCheckBox *cbCovariances;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QRadioButton *rbGauss;
    QRadioButton *rbLevenberg;
    QPushButton *btnInitialGuess;
    QPushButton *btnOptimize;
    QPushButton *btnQuit;
    QSpacerItem *verticalSpacer;
    g2o::Slam2DViewer *viewer;
    QMenuBar *menubar;
    QMenu *menuFile;
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
        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_2->addWidget(label);

        spIterations = new QSpinBox(frame);
        spIterations->setObjectName(QStringLiteral("spIterations"));
        spIterations->setMinimum(1);
        spIterations->setMaximum(100);
        spIterations->setValue(10);

        verticalLayout_2->addWidget(spIterations);

        cbCovariances = new QCheckBox(frame);
        cbCovariances->setObjectName(QStringLiteral("cbCovariances"));

        verticalLayout_2->addWidget(cbCovariances);

        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        rbGauss = new QRadioButton(groupBox);
        rbGauss->setObjectName(QStringLiteral("rbGauss"));
        rbGauss->setChecked(true);

        verticalLayout->addWidget(rbGauss);

        rbLevenberg = new QRadioButton(groupBox);
        rbLevenberg->setObjectName(QStringLiteral("rbLevenberg"));

        verticalLayout->addWidget(rbLevenberg);


        verticalLayout_2->addWidget(groupBox);

        btnInitialGuess = new QPushButton(frame);
        btnInitialGuess->setObjectName(QStringLiteral("btnInitialGuess"));

        verticalLayout_2->addWidget(btnInitialGuess);

        btnOptimize = new QPushButton(frame);
        btnOptimize->setObjectName(QStringLiteral("btnOptimize"));

        verticalLayout_2->addWidget(btnOptimize);

        btnQuit = new QPushButton(frame);
        btnQuit->setObjectName(QStringLiteral("btnQuit"));

        verticalLayout_2->addWidget(btnQuit);

        verticalSpacer = new QSpacerItem(20, 364, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout->addWidget(frame);

        viewer = new g2o::Slam2DViewer(centralwidget);
        viewer->setObjectName(QStringLiteral("viewer"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(viewer);

        BaseMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(BaseMainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 23));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        BaseMainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(BaseMainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        BaseMainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionLoad);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);

        retranslateUi(BaseMainWindow);
        QObject::connect(btnQuit, SIGNAL(clicked()), BaseMainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(BaseMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *BaseMainWindow)
    {
        BaseMainWindow->setWindowTitle(QApplication::translate("BaseMainWindow", "g2o SLAM2D Demo", 0));
        actionLoad->setText(QApplication::translate("BaseMainWindow", "Load", 0));
        actionSave->setText(QApplication::translate("BaseMainWindow", "Save", 0));
        actionQuit->setText(QApplication::translate("BaseMainWindow", "Quit", 0));
        label->setText(QApplication::translate("BaseMainWindow", "# Iterations", 0));
        cbCovariances->setText(QApplication::translate("BaseMainWindow", "Covariances", 0));
        groupBox->setTitle(QApplication::translate("BaseMainWindow", "Method", 0));
        rbGauss->setText(QApplication::translate("BaseMainWindow", "Gauss-Newton", 0));
        rbLevenberg->setText(QApplication::translate("BaseMainWindow", "Levenberg", 0));
        btnInitialGuess->setText(QApplication::translate("BaseMainWindow", "Initial Guess", 0));
        btnOptimize->setText(QApplication::translate("BaseMainWindow", "Optimize", 0));
        btnQuit->setText(QApplication::translate("BaseMainWindow", "Quit", 0));
        menuFile->setTitle(QApplication::translate("BaseMainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class BaseMainWindow: public Ui_BaseMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_MAIN_WINDOW_H
