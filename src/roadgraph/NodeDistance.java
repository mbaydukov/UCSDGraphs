package roadgraph;

/**
 * Created by ann on 2/8/16.
 */
public class NodeDistance implements Comparable {

    MapNode mapNode;
    double distance;

    public NodeDistance(MapNode mapNode, double distance) {
        this.mapNode = mapNode;
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public MapNode getMapNode() {
        return mapNode;
    }

    public void setMapNode(MapNode mapNode) {
        this.mapNode = mapNode;
    }

    @Override
    public int compareTo(Object o) {
        NodeDistance nodeDistance = (NodeDistance)o;
        return ((Double)nodeDistance.getDistance()).compareTo(this.getDistance());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        NodeDistance that = (NodeDistance) o;

        if (mapNode != null ? !mapNode.equals(that.mapNode) : that.mapNode != null) return false;

        return true;
    }

    @Override
    public int hashCode() {
        return mapNode != null ? mapNode.hashCode() : 0;
    }
}
