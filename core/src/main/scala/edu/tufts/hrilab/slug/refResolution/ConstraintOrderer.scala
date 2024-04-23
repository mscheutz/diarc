/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.refResolution

import edu.tufts.hrilab.fol.Term

import scala.concurrent.duration.Duration
import scala.util.Random

abstract class ConstraintOrderer {
  def order(a:Property,b:Property): Boolean
}
case class PropertyTemplate(name:String,arity:Int,likelihood:Double,latency:Duration,pFloor:Double,pCeiling:Double,header:String="")

class sizeOrderer extends ConstraintOrderer {
  def argSize(a: Property) =  a.predicateForm.get(0).asInstanceOf[Term].size()

  override def order(a: Property, b: Property) = argSize(a) < argSize(b)
}

class reverseSizeOrderer extends ConstraintOrderer {
  def argSize(a: Property) =  a.predicateForm.get(0).asInstanceOf[Term].size()

  override def order(a: Property, b: Property) = argSize(a) > argSize(b)
}

class randomSizeOrderer extends ConstraintOrderer {
  override def order(a: Property, b: Property) = Random.nextBoolean()
}

class costOrderer(pts:Seq[PropertyTemplate]) extends ConstraintOrderer {
  val costMap:Map[String,Duration] = pts.map(p => p.name -> p.latency).toMap
  def cost(a:Property) = costMap(a.predicateForm.get(0).getName)
  override def order(a: Property, b: Property) = cost(a) < cost(b)
}

class reverseCostOrderer(pts:Seq[PropertyTemplate]) extends ConstraintOrderer {
  val costMap:Map[String,Duration] = pts.map(p => p.name -> p.latency).toMap
  def cost(a:Property) = costMap(a.predicateForm.get(0).getName)
  override def order(a: Property, b: Property) = cost(a) > cost(b)
}

class frequencyOrderer(pts:Seq[PropertyTemplate]) extends ConstraintOrderer {
  val costMap:Map[String,Double] = pts.map(p => p.name -> p.likelihood).toMap
  def cost(a:Property) = costMap(a.predicateForm.get(0).getName)
  override def order(a: Property, b: Property) = cost(a) < cost(b)
}

class reverseFrequencyOrderer(pts:Seq[PropertyTemplate]) extends ConstraintOrderer {
  val costMap:Map[String,Double] = pts.map(p => p.name -> p.likelihood).toMap
  def cost(a:Property) = costMap(a.predicateForm.get(0).getName)
  override def order(a: Property, b: Property) = cost(a) > cost(b)
}