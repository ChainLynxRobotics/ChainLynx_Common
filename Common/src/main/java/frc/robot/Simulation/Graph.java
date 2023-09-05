package frc.robot.Simulation;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import javax.swing.SwingUtilities;

import frc.robot.Control.PID.DoubleFunction;

import javax.swing.JFrame;
import javax.swing.JPanel;
import java.lang.Math;

import java.util.ArrayList;


class Pointp {
    public final double x;
    public final double y;
    public Pointp(double X, double Y) {
        x = X;
        y = Y;
    }
}

/**
 * A 2d Cartesian Graph object that drives graphics for a viewer. Use init() to render
 * and drawagain() to render again. Use addPlot or addPoint to add info, and the colors are from
 * the awt library. The graph has multiple plots, and will auto adjust y window, but the info can be changed
 * with the associated GraphConfig.
 */
public class Graph extends JPanel {
    //todo: add more display features
    private static GraphConfig config;
    private ArrayList<ArrayList<Pointp>> graph;
    
    private JFrame frame;
    private ArrayList<Color> colors;
    private int pxw;
    private int pxh;


    public static void main(String[] args) {
        test();
    }


    public static void test() {
        GraphConfig conf = new Graph.GraphConfig();
        Graph example = new Graph(conf);
        double dt = 0.01;
        example.addPlot(Color.RED);
        for(double t = -3*3.14159; t < 3*3.14159; t+=dt) {
            example.addPoint(t, Math.sin(t), 0);
        }
        example.init(500, 500, "Graph");
    }


    public void addPlot(Color awtcolor) {
        graph.add(new ArrayList<Pointp>());
        colors.add(awtcolor);
    }

    public void addPoint(double x, double y, int plotNumber, double eb, double sb) {
        Pointp p = new Pointp(x, y);
        graph.get(plotNumber).add(p);
        if (p.x > config.x2) {
            extendBy(eb);
        }
        if (p.y > config.y2) {
            config.y2 = p.y;
        }
        if (p.y < config.y1) {
            config.y1 = p.y;
        }
        if (ppsx() < config.minpps) {
            xscaleBy(sb);
        }
        if (ppsy() < config.minpps) {
            yscaleBy(sb);
        }
    }

    public void addPoint(double x, double y, int plotNumber) {
        addPoint(x, y, plotNumber, 5., 2.);
    }

    public void plot(double x1, double x2, DoubleFunction f, double dt, int plotNumber) {
        for (double x = x1; x < x2; x += dt) {
            addPoint(x, f.eval(x), plotNumber);
        }
    }

    public void extendBy(double x) {
        config.x2 += x;
    }
    public void xscaleBy(double x) { //scale only represents the lines in the back showing the values
        config.xScale *= x;
    }
    public void yscaleBy(double y) {
        config.yScale *= y;
    }
    public double ppsx() {
        return pxw * config.xScale / (config.x2 - config.x1); //pixels per window * window per x * x per scale = pixels per scale
    }
    public double ppsy() {
        return pxh * config.yScale / (config.y2 - config.y1); //pixels per window * window per y * y per scale = pixels per scale
    }
    public double map(double a1, double a2, double b1, double b2, double n) {
        return (n - a1) / (a2 - a1) * (b2 - b1) + b1; //don't use b1 and b2 swapped, doesn't work correctly, because adds bottom of range (maybe works? idek)
    }
    public int mapx(double n) {
        return (int)map(config.x1, config.x2, 0, pxw, n);
    }
    public int mapy(double n) {
        return (int)map(config.y1, config.y2, pxh, 0, n); //todo fix this it's upside down but if i upside down it fills :((
    }

    public void addComponentListener() {
        frame.addComponentListener(new ComponentListener() {

            @Override
            public void componentResized(ComponentEvent e) {
                double width = e.getComponent().getWidth();
                double length = e.getComponent().getHeight();
                
                config.x1 = -width/2;
                config.x2 = width/2;
                config.y1 = -length/2;
                config.y2 = length/2;
            }

            @Override
            public void componentMoved(ComponentEvent e) {}

            @Override
            public void componentShown(ComponentEvent e) {}

            @Override
            public void componentHidden(ComponentEvent e) {}
        });

        drawAgain();
    }


    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawGraph(g);
    }

    public void drawAgain() {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                frame.repaint();
            }
        });
    }

    public void drawGraph(Graphics g) {
        //TODO: make stroke thickness somehow
        double x1 = config.x1;
        double x2 = config.x2;
        double y1 = config.y1;
        double y2 = config.y2;
        double xratio = x1 / (x1 - x2);
        double yratio = y1 / (y1 - y2);
        var center = new Point((int)(pxw * xratio), pxh - (int)(pxh * yratio));
        
        g.setColor(Color.GRAY.brighter());
        if (x2 > 0) {
            for (int i = 0; i < x2; i += config.xScale) {
                int x = mapx(i);
                g.drawLine(x,0,x,pxh);
            }
        }
        if (x1 < 0) {
            for (int i = 0; i > x1; i -= config.xScale) {
                int x = mapx(i);
                g.drawLine(x,0,x,pxh);
            }
        }
        if (y2 > 0) {
            for (int i = 0; i < y2; i += config.yScale) {
                int y = mapy(i);
                g.drawLine(0,y,pxw,y);
            }
        }
        if (y1 < 0) {
            for (int i = 0; i > y1; i -= config.yScale) {
                int y = mapy(i);
                g.drawLine(0,y,pxw,y);
            }
        }

        g.setColor(Color.BLACK);
        g.drawLine(center.x, 0, center.x, pxh);
        g.drawLine(0, center.y, pxw, center.y);
        for (int j = 0; j < graph.size(); j++) {
            g.setColor(colors.get(j));
            for (int i = 1; i < graph.get(j).size(); i++) {
                g.drawLine(mapx(graph.get(j).get(i-1).x), mapy(graph.get(j).get(i-1).y), mapx(graph.get(j).get(i).x), mapy(graph.get(j).get(i).y));
            }
        }
    }

    public void init(int pxw, int pxh, String name) {
        this.pxw = pxw;
        this.pxh = pxh;
        this.frame = new JFrame(name);
        this.frame.setSize(pxw, pxh);
        this.frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.frame.getContentPane().add(this, BorderLayout.CENTER);
        this.frame.setVisible(true);
    }    

    public JFrame getJFrame() {
        return frame;
    }

    public static class GraphConfig {
        public double x1; //currently UB if x1 > x2, same with y
        public double x2;
        public double y1;
        public double y2;
        public double xScale;
        public double yScale;
        public double minpps = 5; //minimum pixels per scale
        public int thickness = 3;

        public GraphConfig(double x1, double x2, double y1, double y2, double xScale, double yScale, double minpps) {
            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
            this.xScale = xScale;
            this.yScale = yScale;
            // this.xCenter = mapx(0) 
            // this.yCenter = mapy(0)
        }
        public GraphConfig() {
            this(-10., 10., -10., 10., 1., 1., 5.);
        }
        public static class Builder {
            private double x1; //currently UB if x1 > x2, same with y
            private double x2;
            private double y1;
            private double y2;
            private double xScale;
            private double yScale;
            private double minpps = 5; //minimum pixels per scale
            private int thickness = 3;
            public Builder() {
                this.x1 = -10.;
                this.x2 = 10.;
                this.y1 = -10.;
                this.y2 = 10.;
                this.xScale = 1.;
                this.yScale = 1.;
            }
            public Builder x1(double x1) {
                this.x1 = x1;
                return this;
            }
            public Builder x2(double x2) {
                this.x2 = x2;
                return this;
            }
            public Builder y1(double y1) {
                this.y1 = y1;
                return this;
            }
            public Builder y2(double y2) {
                this.y2 = y2;
                return this;
            }
            public Builder xScale(double xScale) {
                this.xScale = xScale;
                return this;
            }
            public Builder yScale(double yScale) {
                this.yScale = yScale;
                return this;
            }
            public Builder minpps(double minpps) {
                this.minpps = minpps;
                return this;
            }
            public Builder thickness(int thickness) {
                this.thickness = thickness;
                return this;
            }
            public GraphConfig build() {
                return new GraphConfig(x1, x2, y1, y2, xScale, yScale, minpps);
            }
        }
    }

    public Graph(GraphConfig c) { //scale only represents the lines in the back showing the values
        this.config = c;
        this.graph = new ArrayList<ArrayList<Pointp>>();
        this.colors = new ArrayList<Color>();
        this.frame = getJFrame();
        this.pxw = 500;
        this.pxh = 500;
    }
}

