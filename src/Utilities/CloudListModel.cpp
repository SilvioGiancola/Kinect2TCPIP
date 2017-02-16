#include "CloudListModel.h"
#include "ui_CloudListModel.h"






void CloudListModel::addCloud(PointCloudT::Ptr PC)
{
    QStandardItem* myCloudItem = new QStandardItem();
    myCloudItem->setCheckable(true);

    for(int i = 0; i < PROP_NBPROP; i++)
    {
        QList<QStandardItem*> myCloudProp;
        myCloudProp.append(new QStandardItem("Name"));
        myCloudProp.append(new QStandardItem("Value"));
        myCloudItem->appendRow(myCloudProp);
    }

    QList<QStandardItem*> myCloudItem2;
    myCloudItem2.append(myCloudItem);
    myCloudItem2.append(new QStandardItem());

    invisibleRootItem()->appendRow(myCloudItem2);


    CustomPointCloud myCustomPC(PC);
    _PCList.append(myCustomPC);
}


/* QStandardItemModel - Constructor
 * This is the construtor where the only variable selectedPC is initialized */
CloudListModel::CloudListModel(QObject *parent) : QStandardItemModel(parent)
{
    //selectedPC.reset(new PointCloudT());

    //undoStack = new QUndoStack(this);
}


/* QStandardItemModel - Count Rows
 * This return the number of opened point clouds, in order to define the number of rows */
int CloudListModel::rowCount(const QModelIndex &parent) const
{
    if(parent == invisibleRootItem()->index())
        return _PCList.size();
    else
        return PROP_NBPROP;
}


/* QStandardItemModel - Get Data
 *
 * return the data to show in the tree
 *
 */
QVariant CloudListModel::data(const QModelIndex &index, int role) const
{

    // in case of display and edition of the data:
    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {

        // First Level: it correspond to the name of the point cloud
        if (index.parent() == invisibleRootItem()->index())
        {
            if (index.column() == 0 )        return QString("Frame ID");
            else if (index.column() == 1 && role == Qt::DisplayRole )   return QString::fromStdString(_PCList.at(index.row()).PC->header.frame_id).section("/", -1,-1);
            else if (index.column() == 1 && role == Qt::EditRole )   return QString::fromStdString(_PCList.at(index.row()).PC->header.frame_id);
        }

        // Second Level: it correspond to the parameters of the point cloud
        else if (index.parent().parent() == invisibleRootItem()->index())
        {

            QModelIndex PCindex = index.parent();
            CustomPointCloud thisPointCloud = _PCList.at(PCindex.row());
            // Intrinsic parameters
            if (index.row() == PROP_NUMBER && index.column() == 0)    return QString("Number");
            if (index.row() == PROP_NUMBER && index.column() == 1)    return QLocale(QLocale::German).toString((int)thisPointCloud.size());

            if (index.row() == PROP_ORGANIZED && index.column() == 0)   return QString("Organized");
            if (index.row() == PROP_ORGANIZED && index.column() == 1)
            {
                if ( thisPointCloud.isOrganized() ) return QString("w:%1 x h:%2").arg(thisPointCloud.width()).arg(thisPointCloud.height());
                else return false;
            }

            if (index.row() == PROP_DENSE && index.column() == 0)       return QString("Dense");
            if (index.row() == PROP_DENSE && index.column() == 1)       return (bool) ( thisPointCloud.isDense());


            // Visualization parameters
           /* double point_size = 1, opacity = 1;
            if (_viewer->contains(PointCloudTthisPointCloud->header.frame_id))
            {
                _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, thisPointCloud->header.frame_id);
                _viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, thisPointCloud->header.frame_id);
            }*/

            if (index.row() == PROP_SIZE && index.column() == 0)        return QString("Size");
            if (index.row() == PROP_SIZE && index.column() == 1)        return thisPointCloud.getPointSize();

            if (index.row() == PROP_OPACITY && index.column() == 0)     return QString("Opacity");
            if (index.row() == PROP_OPACITY && index.column() == 1)     return thisPointCloud.getOpacity()*10;

            if (index.row() == PROP_NORMAL && index.column() == 0)      return QString("Normals");
            if (index.row() == PROP_NORMAL && index.column() == 1)      return QVariant();
        }

    }


    // This define the font for every index displayed
    else if (role == Qt::FontRole){}


    // This define the alignement for the index displayed
    else if (role ==  Qt::TextAlignmentRole) {}


    // This define if the index can be and is checked
    else if (role ==  Qt::CheckStateRole)
    {
        // This correspond to the first level
        if ( index.parent() == invisibleRootItem()->index() )
        {
            // on which I only care about the first column
            if ( index.column() == 0 ) // If PointCloudItem
            {
                //if (_viewer->contains(_PCList.at(index.row())->header.frame_id))        return Qt::Checked;
              //  else if (!_viewer->contains(_PCList.at(index.row())->header.frame_id))  return Qt::Unchecked;
                if (_PCList.at(index.row()).isVisible)        return Qt::Checked;
                else return Qt::Unchecked;
            }
        }

        // Second Level: it correspond to the parameters of the point cloud
        else if (index.parent().parent() == invisibleRootItem()->index())
        {
           if (index.row() == PROP_NORMAL && index.column() == 0)
            {
                if (_PCList.at(index.parent().row()).isNormalVisible)        return Qt::Checked;
                else return Qt::Unchecked;
            }
        }
    }

    return QVariant();
}


/* QStandardItemModel - Set Data
 *
 * set the data shown in the tree when occur an edition
 *
 */
bool CloudListModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
    // in case of edition of my datas
    if (role == Qt::EditRole)
    {
        // Only the First column can be edited
        if ( index.column() == 1 )
        {
            // First Level : Corespond to the point cloud ID
            if ( index.parent() == invisibleRootItem()->index() )
            {
                // I remove the old point cloud
                //removePointCloudFromViewer(_PCList.at(index.row()));

                // I change its name
                _PCList.at(index.row()).PC->header.frame_id = value.toString().toStdString();

                // I add the point cloud with its new name
               // addPointCloudToViewer(_PCList.at(index.row()), false, true);
            }

            // Second level: correspond to the properties
            else if ( index.parent().parent() == invisibleRootItem()->index() ) // Second Level
            {
                QModelIndex PCindex = index.parent();
                if (index.row() == PROP_DENSE)
                    _PCList.at(PCindex.row()).PC->is_dense = value.toBool();
//
             //   else if (index.row() == PROP_SIZE)
             //       _PCList.at(PCindex.row()).pointSize = ( (value.toInt()>0)?value.toInt():1 );

                //else if (index.row() == PROP_OPACITY )
                    //_PCList.at(PCindex.row()).setOpacity( value.toDouble()/10 );


              /*  else if (index.row() == PROP_SIZE && _viewer->contains(_PCList.at(PCindex.row())->header.frame_id))
                    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (value.toInt()>0)?value.toInt():1, _PCList.at(PCindex.row())->header.frame_id);

                else if (index.row() == PROP_OPACITY && _viewer->contains(_PCList.at(PCindex.row())->header.frame_id))
                    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, value.toDouble()/10, _PCList.at(PCindex.row())->header.frame_id);
*/

            }
            //emit updateViewer();
        }
    }


    // Handle the checkboxes
    else if (role == Qt::CheckStateRole)
    {
        // Check the first column only
        if ( index.column() == 0 ) // If PointCloudItem
        {
            // First Level: correspond to the Point Clouds
            if ( index.parent() == invisibleRootItem()->index() )
            {

                  // If the checkbox is checked, I add the point cloud
             //   if (value == Qt::Checked)
                    //addPointCloudToViewer(_PCList.at(index.row()),false);

                // If the checkbox is unchecked, I remove the point cloud
              //  else if  (value == Qt::Unchecked)
               //     removePointCloudFromViewer(_PCList.at(index.row()));

                emit dataChanged(index.child(0,0),index.child(PROP_NBPROP,1));
            }



            // Second level: correspond to the properties
            else if ( index.parent().parent() == invisibleRootItem()->index() ) // Second Level
            {
                if ( index.row() == PROP_NORMAL )
                {
                    // If the checkbox is checked, I add the Normal to the existing PC
                  /*  if (value == Qt::Checked)
                        addPointCloudToViewer(_PCList.at(index.parent().row()));

                    // If the checkbox is unchecked, I remove only the Normal PC
                    else if  (value == Qt::Unchecked)
                        if (_viewer->contains(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal))
                        {
                            _viewer->removePointCloud(_PCList.at(index.parent().row())->header.frame_id + SuffixNormal);
                            emit updateViewer();
                        }*/
                }
            }

        }

    }

    return true;
}


/* QStandardItemModel - Set Flags
 *
 * The Flag handles the index properties (Enable, Selectable, Checkable, Editable)
 *
 */
Qt::ItemFlags CloudListModel::flags(const QModelIndex & index) const
{
    // First level: the point cloud
    if ( index.parent() == invisibleRootItem()->index())
    {
        if ( index.column() == 0 )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
        if ( index.column() == 1 )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
    }

    // Second Level the Properties
    else if ( index.parent().parent() == invisibleRootItem()->index())
    {
        // First column
        if (index.column() == 0 )
        {
            if (index.row() == PROP_NORMAL )    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
            else                                return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
        }

        // Second Column
        if (index.column() == 1 )
        {
            if (index.row() == PROP_NORMAL)     return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_NUMBER)     return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_ORGANIZED)  return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
            if (index.row() == PROP_DENSE)      return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
            if (index.row() == PROP_SIZE)       return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
            if (index.row() == PROP_OPACITY)    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
        }

    }

    return Qt::ItemFlags();
}


/* QStandardItemModel - Set Headers
 *
 * The Headers are the titles given to the column and rows
 *
 */
QVariant CloudListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    // Only the display role is handles
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal)
        {
            if (section == 0)   return QString("Name");
            if (section == 1)   return QString("Value");
        }

        if (orientation == Qt::Vertical)
            return section;
    }
    return QVariant();
}


