/********************************************************************************
** Form generated from reading UI file 'base_properties_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASE_PROPERTIES_WIDGET_H
#define UI_BASE_PROPERTIES_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_BasePropertiesWidget
{
public:
    QVBoxLayout *verticalLayout;
    QTableWidget *tableWidget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnApply;
    QPushButton *btnOK;
    QPushButton *btnCancel;

    void setupUi(QWidget *BasePropertiesWidget)
    {
        if (BasePropertiesWidget->objectName().isEmpty())
            BasePropertiesWidget->setObjectName(QStringLiteral("BasePropertiesWidget"));
        BasePropertiesWidget->resize(543, 443);
        verticalLayout = new QVBoxLayout(BasePropertiesWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tableWidget = new QTableWidget(BasePropertiesWidget);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));

        verticalLayout->addWidget(tableWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(428, 23, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        btnApply = new QPushButton(BasePropertiesWidget);
        btnApply->setObjectName(QStringLiteral("btnApply"));

        horizontalLayout->addWidget(btnApply);

        btnOK = new QPushButton(BasePropertiesWidget);
        btnOK->setObjectName(QStringLiteral("btnOK"));

        horizontalLayout->addWidget(btnOK);

        btnCancel = new QPushButton(BasePropertiesWidget);
        btnCancel->setObjectName(QStringLiteral("btnCancel"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btnCancel->sizePolicy().hasHeightForWidth());
        btnCancel->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(btnCancel);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(BasePropertiesWidget);
        QObject::connect(btnCancel, SIGNAL(clicked()), BasePropertiesWidget, SLOT(close()));

        QMetaObject::connectSlotsByName(BasePropertiesWidget);
    } // setupUi

    void retranslateUi(QWidget *BasePropertiesWidget)
    {
        BasePropertiesWidget->setWindowTitle(QApplication::translate("BasePropertiesWidget", "Properties", 0));
        btnApply->setText(QApplication::translate("BasePropertiesWidget", "Apply", 0));
        btnOK->setText(QApplication::translate("BasePropertiesWidget", "OK", 0));
        btnCancel->setText(QApplication::translate("BasePropertiesWidget", "Cancel", 0));
    } // retranslateUi

};

namespace Ui {
    class BasePropertiesWidget: public Ui_BasePropertiesWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_PROPERTIES_WIDGET_H
