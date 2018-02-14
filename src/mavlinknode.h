#ifndef MAVLINKNODE_H
#define MAVLINKNODE_H

#include <QMainWindow>

namespace Ui {
class MavlinkNode;
}

class MavlinkNode : public QMainWindow
{
    Q_OBJECT

public:
    explicit MavlinkNode(QWidget *parent = 0);
    ~MavlinkNode();

private:
    Ui::MavlinkNode *ui;
};

#endif // MAVLINKNODE_H
