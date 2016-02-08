package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class MapNode {
    private GeographicPoint location;
    private Set<MapEdge> edges;

    public MapNode(GeographicPoint location) {
        this.location = location;
        this.edges = new HashSet<>();
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public Set<MapEdge> getEdges() {
        return edges;
    }

    public void setEdges(Set<MapEdge> edges) {
        this.edges = edges;
    }

    public void addEdges(Set<MapEdge> edges){
        this.edges.addAll(edges);
    }

    public void addEdge(MapEdge edge){
        this.edges.add(edge);
    }

    public Set<MapNode> getNeighbours(){
        return edges.stream().map(edge -> edge.getTo()).collect(Collectors.toSet());
    }


    /** Returns whether two nodes are equal.
     */
    @Override
    public boolean equals(Object o){
        if (!(o instanceof MapNode) || (o == null)) {
            return false;
        }
        MapNode node = (MapNode)o;
        return node.location.equals(this.location);
    }

    /** The same HashCode as in GeographicalPoint.
     */
    @Override
    public int hashCode(){
        return location.hashCode();
    }
}
