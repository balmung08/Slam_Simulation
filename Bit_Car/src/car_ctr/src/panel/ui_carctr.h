/********************************************************************************
** Form generated from reading UI file 'carctr.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CATCTR_H
#define UI_CATCTR_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Panel_CarCtr
{
public:
    QLabel *Label_Wheel1;
    QLabel *label;
    QLabel *Label_Wheel2;
    QLabel *Label_Wheel3;
    QLabel *Label_Wheel4;
    QLabel *Label_Turn1;
    QLabel *Label_Turn2;
    QLabel *Label_Battery;

    void setupUi(QWidget *Panel_CarCtr)
    {
        if (Panel_CarCtr->objectName().isEmpty())
            Panel_CarCtr->setObjectName(QString::fromUtf8("Panel_CarCtr"));
        Panel_CarCtr->resize(346, 400);
        Label_Wheel1 = new QLabel(Panel_CarCtr);
        Label_Wheel1->setObjectName(QString::fromUtf8("Label_Wheel1"));
        Label_Wheel1->setGeometry(QRect(20, 50, 311, 40));
        QFont font;
        font.setPointSize(14);
        Label_Wheel1->setFont(font);
        label = new QLabel(Panel_CarCtr);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 10, 261, 20));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font1.setPointSize(15);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);
        Label_Wheel2 = new QLabel(Panel_CarCtr);
        Label_Wheel2->setObjectName(QString::fromUtf8("Label_Wheel2"));
        Label_Wheel2->setGeometry(QRect(20, 95, 311, 40));
        Label_Wheel2->setFont(font);
        Label_Wheel3 = new QLabel(Panel_CarCtr);
        Label_Wheel3->setObjectName(QString::fromUtf8("Label_Wheel3"));
        Label_Wheel3->setGeometry(QRect(20, 144, 311, 40));
        Label_Wheel3->setFont(font);
        Label_Wheel4 = new QLabel(Panel_CarCtr);
        Label_Wheel4->setObjectName(QString::fromUtf8("Label_Wheel4"));
        Label_Wheel4->setGeometry(QRect(20, 190, 311, 40));
        Label_Wheel4->setFont(font);
        Label_Turn1 = new QLabel(Panel_CarCtr);
        Label_Turn1->setObjectName(QString::fromUtf8("Label_Turn1"));
        Label_Turn1->setGeometry(QRect(20, 239, 311, 40));
        Label_Turn1->setFont(font);
        Label_Turn2 = new QLabel(Panel_CarCtr);
        Label_Turn2->setObjectName(QString::fromUtf8("Label_Turn2"));
        Label_Turn2->setGeometry(QRect(20, 290, 301, 40));
        Label_Turn2->setFont(font);
        Label_Battery = new QLabel(Panel_CarCtr);
        Label_Battery->setObjectName(QString::fromUtf8("Label_Battery"));
        Label_Battery->setGeometry(QRect(20, 340, 301, 40));
        Label_Battery->setFont(font);

        retranslateUi(Panel_CarCtr);

        QMetaObject::connectSlotsByName(Panel_CarCtr);
    } // setupUi

    void retranslateUi(QWidget *Panel_CarCtr)
    {
        Panel_CarCtr->setWindowTitle(QApplication::translate("Panel_CarCtr", "\345\260\217\350\275\246\347\233\221\346\216\247", nullptr));
        Label_Wheel1->setText(QApplication::translate("Panel_CarCtr", "\351\251\261\345\212\250\350\275\2561 ", nullptr));
        label->setText(QApplication::translate("Panel_CarCtr", "\345\260\217\350\275\246\347\233\221\346\216\247", nullptr));
        Label_Wheel2->setText(QApplication::translate("Panel_CarCtr", "\351\251\261\345\212\250\350\275\2562", nullptr));
        Label_Wheel3->setText(QApplication::translate("Panel_CarCtr", "\351\251\261\345\212\250\350\275\2563", nullptr));
        Label_Wheel4->setText(QApplication::translate("Panel_CarCtr", "\351\251\261\345\212\250\350\275\2564", nullptr));
        Label_Turn1->setText(QApplication::translate("Panel_CarCtr", "\345\211\215\350\275\254\345\220\221", nullptr));
        Label_Turn2->setText(QApplication::translate("Panel_CarCtr", "\345\220\216\350\275\254\345\220\221", nullptr));
        Label_Battery->setText(QApplication::translate("Panel_CarCtr", "\347\224\265\346\261\240", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Panel_CarCtr: public Ui_Panel_CarCtr {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATCTR_H
