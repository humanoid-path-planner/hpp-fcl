#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <iostream>
#include <hpp/fcl/collision.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace hpp::fcl;

int main(int argc, char** argv) 
{
  boost::shared_ptr<Box> box0(new Box(1,1,1));
  boost::shared_ptr<Box> box1(new Box(1,1,1));
  GJKSolver_libccd solver;
  Vec3f contact_points;
  FCL_REAL distance;
  Vec3f normal;

  Transform3f tf0, tf1;
  tf0.setIdentity();
  tf0.setTranslation(Vec3f(.9,0,0));
  tf0.setQuatRotation(Quaternion3f(.6, .8, 0, 0));
  tf1.setIdentity();

  bool res = solver.shapeIntersect(*box0, tf0, *box1, tf1, distance, true, &contact_points, &normal);

  cout << "contact points: " << contact_points << endl;
  cout << "signed distance: " << distance << endl;
  cout << "normal: " << normal << endl;
  cout << "result: " << res << endl;
  
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  hpp::fcl::CollisionResult result;
  hpp::fcl::CollisionRequest request(enable_contact, num_max_contacts, false);


  CollisionObject co0(box0, tf0);
  CollisionObject co1(box1, tf1);

  hpp::fcl::collide(&co0, &co1, request, result);
  vector<Contact> contacts;
  result.getContacts(contacts);

  cout << contacts.size() << " contacts found" << endl;
  BOOST_FOREACH(Contact& contact, contacts) {
    cout << "position: " << contact.pos << endl;
  }
}
