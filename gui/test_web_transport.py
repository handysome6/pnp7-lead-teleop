"""Finite preview responses leave the HTTP connection available for controls."""
import http.client
import json
import time
from unittest.mock import patch
import unittest
from types import SimpleNamespace
from gui.server import serve
from gui.session import ControlLoop

class TransportTests(unittest.TestCase):
    def setUp(self):
        self.backend=SimpleNamespace(camera_names=['wrist'], dataset_progress=lambda:None,
                                     latest_jpeg=lambda _: b'\xff\xd8frame\xff\xd9', latest_frame=lambda _:None)
        self.server=serve(ControlLoop(self.backend),host='127.0.0.1',port=0)
        self.addCleanup(self.server.server_close); self.addCleanup(self.server.shutdown)
        self.conn=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=2)
        self.addCleanup(self.conn.close)

    def test_repeated_previews_and_state_share_connection_without_blocking(self):
        for _ in range(20):
            self.conn.request('GET','/preview/wrist')
            r=self.conn.getresponse(); data=r.read()
            self.assertEqual(r.status,200)
            self.assertEqual(r.getheader('Content-Type'),'image/jpeg')
            self.assertEqual(int(r.getheader('Content-Length')),len(data))
            self.assertEqual(r.getheader('Cache-Control'),'no-store')
            self.conn.request('GET','/api/state')
            r=self.conn.getresponse();self.assertEqual(r.status,200);r.read()

    def test_missing_frame_returns_without_waiting(self):
        self.backend.latest_jpeg=lambda _:None
        self.conn.request('GET','/preview/wrist')
        r=self.conn.getresponse();self.assertEqual(r.status,503);r.read()

    def test_unknown_camera_is_rejected(self):
        self.conn.request('GET','/preview/not-a-camera')
        r=self.conn.getresponse();self.assertEqual(r.status,404);r.read()

    def test_delayed_start_request_never_reaches_command_queue(self):
        data=json.dumps({'name':'begin_episode','state_seen_at':time.time()-30})
        self.conn.request('POST','/api/command',data,{'Content-Type':'application/json'})
        r=self.conn.getresponse(); self.assertEqual(r.status,409);r.read()
        self.assertEqual(self.server.loop._commands,[])

    def test_command_that_expires_in_queue_is_not_executed(self):
        loop=self.server.loop
        command=loop.submit('begin_episode',_expires_at=time.time()-1)
        with patch.object(loop,'_apply') as apply:
            loop._drain_commands()
            apply.assert_not_called()
        self.assertIn('已过期',command.error)

if __name__=='__main__': unittest.main()
