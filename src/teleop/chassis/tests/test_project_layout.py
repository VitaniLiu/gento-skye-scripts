import os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import sender,preview

CHASSIS=Path(__file__).resolve().parents[1]
PROJECT=CHASSIS.parents[2]

class LayoutTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory(dir=PROJECT/'tests')
        self.addCleanup(temp.cleanup)
        self.folder=Path(temp.name)
        environment=patch.dict(os.environ,{},clear=False)
        environment.start();self.addCleanup(environment.stop)
        os.environ.pop('CHASSIS_LOGS_DIR',None)
        os.environ.pop('CHASSIS_ROUTES_DIR',None)

    def test_entry_modules_resolve_project_root_from_source_subdirectories(self):
        self.assertEqual(sender.ROOT,CHASSIS)
        self.assertEqual(preview.ROOT,CHASSIS)

    def test_session_and_transport_logs_use_separate_dated_directories(self):
        with patch.object(sender,'ROOT',self.folder):
            session,stream=sender.open_local_file('live');stream.close()
            transport,stream=sender.open_local_file('ssh-stderr');stream.close()
        self.assertEqual(session.parent.parent,self.folder/'logs/teleop/sessions')
        self.assertEqual(transport.parent.parent,self.folder/'logs/teleop/transport')
        self.assertTrue(session.is_file());self.assertTrue(transport.is_file())
        self.assertNotEqual(session,transport)

    def test_manual_routes_use_teleop_routes_directory(self):
        with patch.object(sender,'ROOT',self.folder):
            route,stream=sender.open_route_file();stream.close()
        self.assertEqual(route.parent.parent,self.folder/'logs/teleop/routes')
        self.assertTrue(route.is_file())

    def test_explicit_output_directories_are_honored_separately(self):
        logs=self.folder/'isolated-logs'
        routes=self.folder/'isolated-routes'
        with patch.object(sender,'ROOT',self.folder),patch.dict(os.environ,{
                'CHASSIS_LOGS_DIR':str(logs),'CHASSIS_ROUTES_DIR':str(routes)}):
            session,stream=sender.open_local_file('live');stream.close()
            route,stream=sender.open_route_file();stream.close()
        self.assertEqual(session.parent.parent,logs/'sessions')
        self.assertEqual(route.parent.parent,routes)
        self.assertTrue(session.is_file());self.assertTrue(route.is_file())

if __name__=='__main__':unittest.main()
