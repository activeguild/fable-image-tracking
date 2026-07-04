import { useMemo, useState } from 'react';
import { FableCanvas, FableCamera, ImageTracker, PlanarContent } from '@j1ngzoue/fable-react-three-fiber';
import { createSampleTargetCanvas } from './sampleTarget';
import { createSampleImageCanvas } from './sampleContent';

export function App() {
  const [status, setStatus] = useState('initializing');
  // The sample target is procedural; a real app passes an image URL instead:
  //   <FableCanvas targetImage="/my-target.png" ...>
  const target = useMemo(() => createSampleTargetCanvas(384), []);
  const contentImage = useMemo(() => createSampleImageCanvas(), []);

  return (
    <>
      <FableCanvas
        targetImage={target}
        style={{ width: '100vw', height: '100vh' }}
        onReady={() => setStatus('searching')}
        onError={(err) => setStatus(`error: ${err.message}`)}
      >
        <FableCamera />
        <ImageTracker
          onFound={() => setStatus('tracking')}
          onLost={() => setStatus('searching')}
        >
          {/* Flat media is automatically routed to the pixel-accurate
              homography overlay; meshes ride on the 3D pose. */}
          <PlanarContent source={contentImage} />
          <mesh position={[0, 0, 0.03]}>
            <boxGeometry args={[0.06, 0.06, 0.06]} />
            <meshStandardMaterial color="hotpink" />
          </mesh>
        </ImageTracker>
        <hemisphereLight args={[0xffffff, 0x555566, 2.2]} />
        <directionalLight position={[1, 2, 3]} intensity={1.5} />
      </FableCanvas>
      <div
        id="status"
        style={{
          position: 'fixed',
          left: 12,
          bottom: 12,
          color: '#fff',
          font: '13px system-ui',
          background: 'rgba(0,0,0,0.5)',
          padding: '6px 10px',
          borderRadius: 6,
        }}
      >
        {status}
      </div>
    </>
  );
}
