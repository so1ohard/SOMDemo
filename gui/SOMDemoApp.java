/*
 * SOMDemo.java
 *
 * Created on December 13, 2002, 2:31 PM
 */

package SOMDemo.gui;

import SOMDemo.util.*;
import SOMDemo.CoreClasses.*;

import javax.swing.*;
import java.awt.image.BufferedImage;
import java.util.Vector;
import static javax.swing.JOptionPane.showMessageDialog;
import java.awt.Color;

/**
 *
 * @author  alanter
 */
public class SOMDemoApp extends javax.swing.JFrame {

	private SOMDemo.util.LatticeRenderer renderPanel;

	private javax.swing.JSplitPane jSplitPane1;

    private javax.swing.JButton jButton1;
	private javax.swing.JButton btnRetrain;
	private javax.swing.JButton btnVect;
	private javax.swing.JButton btnNextStep;

	private javax.swing.JPanel ControlsPanel;
	private javax.swing.JPanel jPanel1;

	private javax.swing.JTextArea jFirstColorComponent;
	private javax.swing.JTextArea jSecondColorComponent;
	private javax.swing.JTextArea jThirdColorComponent;

	//private javax.swing.JLabel jLabelResult;

	private int latticeWidth = 50;
	private int latticeHeight = 50;
	
	private SOMTrainer trainer;
	private SOMLattice lattice;
	public static SOMVector targetVec = null;
	private Vector inputVectors;
	
	/** Creates new form SOMDemo */
	public SOMDemoApp() {
		initComponents();
		SOMVector tempVec;
		lattice = new SOMLattice(latticeWidth, latticeHeight);
		renderPanel.registerLattice(lattice);
		trainer = new SOMTrainer();
		inputVectors = new Vector();

		// Make some colors.  Red, Green, Blue, Yellow, Purple, Black,
		// white, and gray
		tempVec = new SOMVector();
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(0));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(0));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(1));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(0));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(1));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(1));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(0));
		tempVec.addElement(new Double(0));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(1));
		tempVec.addElement(new Double(1));
		inputVectors.addElement(tempVec);
		tempVec = new SOMVector();
		tempVec.addElement(new Double(0.5));
		tempVec.addElement(new Double(0.5));
		tempVec.addElement(new Double(0.5));
		inputVectors.addElement(tempVec);
	}

	/** This method is called from within the constructor to
	 * initialize the form.
	 * WARNING: Do NOT modify this code. The content of this method is
	 * always regenerated by the Form Editor.
	 */
    private void initComponents() {//GEN-BEGIN:initComponents
        jSplitPane1 = new javax.swing.JSplitPane();
        jPanel1 = new javax.swing.JPanel();
        renderPanel = new SOMDemo.util.LatticeRenderer();
        ControlsPanel = new javax.swing.JPanel();
        btnRetrain = new javax.swing.JButton();
        btnVect = new javax.swing.JButton();
        jButton1 = new javax.swing.JButton();
        btnNextStep = new javax.swing.JButton();
        jFirstColorComponent = new javax.swing.JTextArea("0.2",1,10);
		jSecondColorComponent = new javax.swing.JTextArea("0.7",1,10);
		jThirdColorComponent = new javax.swing.JTextArea("0.4" , 1,10);
		//jLabelResult = new JLabel("Nearest cluster");

        setTitle("SOM Demo");
        setResizable(false);
        addWindowListener(new java.awt.event.WindowAdapter() {
            public void windowClosing(java.awt.event.WindowEvent evt) {
                exitForm(evt);
            }
        });

        jSplitPane1.setDividerLocation(400);
        jSplitPane1.setDividerSize(5);
        jSplitPane1.setEnabled(false);
        jPanel1.setLayout(new java.awt.CardLayout());

        renderPanel.setBackground(new java.awt.Color(0, 0, 0));
        renderPanel.setFont(new java.awt.Font("Dialog", 0, 11));
        renderPanel.setMinimumSize(new java.awt.Dimension(200, 200));
        jPanel1.add(renderPanel, "card2");

        jSplitPane1.setLeftComponent(jPanel1);

        btnRetrain.setText("Retrain Map");
        btnRetrain.setEnabled(false);
        btnRetrain.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnRetrainActionPerformed(evt);
            }
        });

        ControlsPanel.add(btnRetrain);

        jButton1.setText("Stop Training");
        jButton1.setEnabled(false);
        jButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton1ActionPerformed(evt);
            }
        });

        ControlsPanel.add(jButton1);

		jFirstColorComponent.setBackground(Color.WHITE);
		ControlsPanel.add(jFirstColorComponent);

		jSecondColorComponent.setBackground(Color.WHITE);
		ControlsPanel.add(jSecondColorComponent);

		jThirdColorComponent.setBackground(Color.WHITE);
		ControlsPanel.add(jThirdColorComponent);


		btnVect.setText("Input Vect");
		btnVect.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				btnVectActionPerformed(evt);
			}
		});

		ControlsPanel.add(btnVect);

        btnNextStep.setText("NextStep");
        btnNextStep.setEnabled(false);
        btnNextStep.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                btnNextStepActionPerformed(evt);
            }
        });

        ControlsPanel.add(btnNextStep);

		//ControlsPanel.add(jLabelResult);

        jSplitPane1.setRightComponent(ControlsPanel);

        getContentPane().add(jSplitPane1, java.awt.BorderLayout.CENTER);

        pack();
        java.awt.Dimension screenSize = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
        setSize(new java.awt.Dimension(550, 400));
        setLocation((screenSize.width-550)/2,(screenSize.height-400)/2);
    }//GEN-END:initComponents

	private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
		trainer.stop();
		btnNextStep.setEnabled(false);
	}//GEN-LAST:event_jButton1ActionPerformed

	private void btnRetrainActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_btnRetrainActionPerformed
		trainer.stop();
		lattice = new SOMLattice(latticeWidth, latticeHeight);
		trainer.setTraining(lattice, inputVectors, renderPanel);
		renderPanel.registerLattice(lattice);
		trainer.start();
		jButton1.setEnabled(true);
		btnNextStep.setEnabled(true);
	}//GEN-LAST:event_btnRetrainActionPerformed
//Тут дёргаем данные из текс филдов
	private void btnVectActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
        float x = 0.0F,y = 0.0F, z = 0.0F;
        try {
           x = Float.parseFloat(jFirstColorComponent.getText());
        } catch (Exception e){
            showMessageDialog(null, "Conversion error");
            return;
        }
        try {
            y = Float.parseFloat(jSecondColorComponent.getText());
        } catch (Exception e){
            showMessageDialog(null, "Conversion error");
            return;
        }
        try {
            z = Float.parseFloat(jThirdColorComponent.getText());
        } catch (Exception e){
            showMessageDialog(null, "Conversion error");
            return;
        }
        try {
            Color color = new Color(x, y, z);
            btnVect.setBackground(color);
            targetVec = new SOMVector();
            targetVec.addElement(x);
            targetVec.addElement(y);
            targetVec.addElement(z);
            btnRetrain.setEnabled(true);
            //showMessageDialog(null, Double.toString(x));
        }catch (Exception e){
            showMessageDialog(null,"Color Error");
            return;
        }
	}

    private void btnNextStepActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
        //WorkState.State = 1;
        trainer.nextStep();
    }

        /** Exit the Application */
	private void exitForm(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_exitForm
		System.exit(0);
	}//GEN-LAST:event_exitForm

	/**
	 * @param args the command line arguments
	 */
	public static void main(String args[]) {
		SOMDemoApp theApp = new SOMDemoApp();
		theApp.show();
		theApp.go();
		//new SOMDemoApp().show();
	}

	public void go() {
		BufferedImage i = renderPanel.getImage();
		renderPanel.registerLattice(lattice);
		renderPanel.render(lattice, 0, inputVectors, "#");
		trainer.setTraining(lattice, inputVectors, renderPanel);
//		trainer.start();
	}

	public void setResultCluster(int x)
	{
		//jLabelResult.setText("Nearest cluster: )" + String.valueOf(x));
	}

    // Variables declaration - do not modify//GEN-BEGIN:variables

    // End of variables declaration//GEN-END:variables

}