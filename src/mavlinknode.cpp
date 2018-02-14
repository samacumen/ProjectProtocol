#include "mavlinknode.h"
#include "ui_mavlinknode.h"

MavlinkNode::MavlinkNode(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MavlinkNode)
{
    ui->setupUi(this);
}

MavlinkNode::~MavlinkNode()
{
    delete ui;
}
